#!/usr/bin/env python3
'''TOF Sensor reader'''

import datetime
import logging
from pathlib import Path
import re
import sys
import signal
import threading
import time

import ntcore
import wpiutil
from wpiutil.log import DoubleLogEntry, StringLogEntry, DoubleArrayLogEntry

from .cd3 import CornerDetector
from .sensor import TofSensor, SetPins

__version__ = "0.2"

LOG_DIR = Path("logs")

DEFAULT_TOF_ADDRESS = 0x29
DEFAULT_TIMING = 20
DEFAULT_INTER = 25

# Name constants to avoid typos and duplication.
# These are used for both NetworkTables and DataLog, for consistency and simplicity.
class Name:
    DIST_MM = "/Pi/dist_mm"
    TS_DIST_MM = "/Pi/ts_dist_mm"
    CORNERS = "/Pi/Corners" # note inconsistent capitalization
    CORNER = "/Pi/corner"
    CORNER_TS = "/Pi/corner_ts"
    CORNER_DIST_MM = "/Pi/corner_dist_mm"
    TOF_MODE = "/Pi/tof_mode"
    CHUTE_MODE = "/Pi/chute_mode"
    # untested:
    MATCH_TIME = "/AdvantageKit/DriverStation/MatchTime"
    FMS_ATTACHED = "/AdvantageKit/DriverStation/FMSAttached"
    ENABLED = "/AdvantageKit/DriverStation/Enabled"
    
class EntryType:
    DOUBLE = "double"
    STRING = "string"
    DOUBLE_ARRAY = "double_array"


class LogManager:
    """
    Manages DataLogBackgroundWriter for logging TOF sensor data
    """
    def __init__(self, log_dir=LOG_DIR):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True, parents=True)
        self.log = logging.getLogger("tof.log")
        self.writer = None
        
        # Initialize entries dict to track all created entries
        self.entries = {}
        
        # Define initial entries configuration - centralized definition
        self.entry_config = {
            Name.DIST_MM: {"type": EntryType.DOUBLE},
            Name.TS_DIST_MM: {"type": EntryType.DOUBLE_ARRAY},
            Name.CORNER: {"type": EntryType.DOUBLE},
            Name.CORNER_DIST_MM: {"type": EntryType.DOUBLE},
            Name.TOF_MODE: {"type": EntryType.STRING},
            Name.CHUTE_MODE: {"type": EntryType.STRING},
        }
        
        # Create initial log file
        self.start_new_log()
    
    def start_new_log(self):
        """Start a new log file with timestamp-based name"""
        timestamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S")
        filename = f"tof-{timestamp}.wpilog"
        self.log.info(f"Starting new log file: {filename}")
        
        # If we have an existing writer, pause it before switching
        if self.writer:
            self.writer.pause()
        
        # Create new writer
        self.writer = wpiutil.DataLogBackgroundWriter(
            str(self.log_dir),  # Directory 
            filename,           # Filename
            period=0.5,         # Flush period in seconds
            extraHeader="TOF Sensor Data"
        )
        
        # Re-create all entries with the new writer
        self._recreate_entries()
        
        return filename
    
    def _recreate_entries(self):
        """Recreate all log entries with the new writer"""
        # Clear current entries
        self.entries = {}
        
        # Create each entry from the configuration
        for name, config in self.entry_config.items():
            self._create_entry(
                name, 
                config["type"],
                metadata=config.get("metadata", None)
            )
    
    def _create_entry(self, name, entry_type, metadata=None):
        """Internal method to create a new log entry"""
        if entry_type == EntryType.DOUBLE:
            if metadata:
                entry = DoubleLogEntry(self.writer, name, metadata)
            else:
                entry = DoubleLogEntry(self.writer, name)
        elif entry_type == EntryType.STRING:
            if metadata:
                entry = StringLogEntry(self.writer, name, metadata)
            else:
                entry = StringLogEntry(self.writer, name)
        elif entry_type == EntryType.DOUBLE_ARRAY:
            if metadata:
                entry = DoubleArrayLogEntry(self.writer, name, metadata)
            else:
                entry = DoubleArrayLogEntry(self.writer, name)
        else:
            raise ValueError(f"Unsupported entry type: {entry_type}")
            
        # Store entry for future use
        self.entries[name] = entry
        
        return entry
    
    def append_double(self, name, value, timestamp):
        """Append a double value to the specified entry"""
        if name not in self.entries:
            self.log.warning(f"Entry {name} does not exist")
            return False
            
        try:
            self.entries[name].append(value, int(timestamp * 1000000))
            return True
        except Exception as e:
            self.log.error(f"Error appending to {name}: {e}")
            return False
    
    def append_string(self, name, value, timestamp):
        """Append a string value to the specified entry"""
        if name not in self.entries:
            self.log.warning(f"Entry {name} does not exist")
            return False
            
        try:
            self.entries[name].append(value, int(timestamp * 1000000))
            return True
        except Exception as e:
            self.log.error(f"Error appending to {name}: {e}")
            return False
    
    # def append_double_array(self, name, value, timestamp):
    #     """Append a double array value to the specified entry"""
    #     if name not in self.entries:
    #         self.log.warning(f"Entry {name} does not exist")
    #         return False
            
    #     try:
    #         self.entries[name].append(value, int(timestamp * 1000000))
    #         return True
    #     except Exception as e:
    #         self.log.error(f"Error appending to {name}: {e}")
    #         return False
    
    def close(self):
        """Close the log writer"""
        if self.writer:
            self.writer.stop()
            self.writer = None


class SensorManager:
    """
    Open and start the VL53L1X ranging sensor
    """

    def __init__(self, address=DEFAULT_TOF_ADDRESS):
        self.address = address
        
        # This will raise OSError if there's no /dev/i2c-1.
        self.tof = TofSensor(bus=1, address=address)
        self.log = logging.getLogger(f'tof.{address:x}')
        self.enabled = True


    def configure(self, timing, inter):
        # this will fail if there's nothing on the bus
        self.tof.init(True)  # True means set for 2.8V

        self.log.debug('set mode')
        self.tof.set_long_distance_mode(False)  # short distance mode for faster readings

        # Lower timing budgets allow for faster updates, but sacrifice accuracy
        self.log.debug('set timing=%s ms', timing)
        self.tof.set_timing_budget_ms(timing)

        # inter-measurement period must be higher than timing budget so use max()
        self.log.debug('set inter=%s ms', inter)
        self.tof.set_inter_measurement_period_ms(max(timing, inter))

        self.log.debug('start streaming')
        self.tof.start_streaming()


    def is_running(self):
        return self.tof.is_running()


    def read(self, timing, inter, callback=None):
        '''Run loop reading from the sensor, via a blocking call to
        get_reading(), which retrieves readings from the sensor thread.'''
        try:
            self.log.info('reading: timing=%s inter=%s', timing, inter)
            ts_last = time.monotonic() # try to match the one Rust uses...
            self.enabled = True
            running = False
            warned = None # flag when we've reported a problem, to avoid spamming
            while self.enabled:
                if not running:
                    # Try reconfiguring sensor.  If it's present it will
                    # end up configured properly and streaming.  If it's
                    # not present or there's a problem, we'll get an
                    # exception, then try again after a brief pause.
                    try:
                        self.configure(timing, inter)
                    except Exception as ex:
                        if not warned or warned is not ex.__class__:
                            warned = ex.__class__
                            self.log.error("Error: %s", ex)

                        time.sleep(0.2)
                        continue
                    running = True
                    warned = False

                    self.log.info("ACTIVATED")

                try:
                    # This should block until there's a reading or error.
                    reading = self.tof.get_reading()

                    if reading is None:
                        running = False
                        self.log.warning("DEACTIVATED")
                        continue

                except Exception as ex:
                    if not warned:
                        warned = True
                        self.log.error("Error: %s", ex)

                    running = False
                    continue

                else:
                    ts, distance, status = reading
                    # self.log.debug('reading: %s,%s,%s', ts, distance, status)

                    delta = ts - ts_last
                    ts_last = ts

                    if callback is not None:
                        callback(ts, distance, status, delta)

                    time.sleep(0.001)  # Small sleep to prevent CPU spinning
                    # (although the new Rust code should block us anyway
                    # so in theory this is no longer required)

        finally:
            self.tof.stop_streaming()


    def shutdown(self):
        self.enabled = False
        self.tof.stop_streaming()


def get_args():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", action="store_true")
    parser.add_argument("--debug-cd", action="store_true")
    parser.add_argument("--roi", default="0,15,15,0",
        help="region of interest (default %(default)s)")
    parser.add_argument(
        "-t", "--timing", type=int, default=DEFAULT_TIMING,
        help="timing budget in ms (default %(default)s)")
    parser.add_argument(
        "-i", "--inter", type=int, default=DEFAULT_INTER,
        help="inter-measurement period in ms (default %(default)s)")
    parser.add_argument("--serve", action="store_true",
        help="serve NT instance (for testing)")
    parser.add_argument("--stdout", action="store_true")
    parser.add_argument("--slope", type=float, default=400,
        help="slope threshold (for corner detector) (default %(default)s)")
    parser.add_argument("--speed", type=float, default=450,
        help="fake speed (mm/s) (for testing, default %(default)s)")
    parser.add_argument("--log-cycle-time", type=int, default=0,
        help="Cycle log files after specified minutes (0=disabled)")

    args = parser.parse_args()
    args.roi = tuple(int(x) for x in args.roi.split(","))
    # Region of interest is top-left X, top-left Y, bottom-right X, bottom-right Y.
    # Minimum size is 4x4, values must be within 0-15 (inclusive)
    if abs(args.roi[0] - args.roi[2]) < 3 or abs(args.roi[1] - args.roi[3]) < 3:
        print("ROI must be at least width and height 4")
        sys.exit(0)

    return args


class TofMain:
    def __init__(self, args):
        self.args = args
        self.log = logging.getLogger("tof")
        self.running = False
        self.speed = self.args.speed
        self.tof_mode = 'none'
        self.saw_corner = False

        self.tof_mode_sub = None
        self.tof_mode_pub = None
        self.chute_mode_sub = None
        self.nt = None
        self.mode_poller = None
        self.dist_pub = None
        self.ts_dist_pub = None
        self.corner_pub = None
        self.corner_ts_pub = None
        
        # Initialize log manager
        self.log_manager = LogManager()
        
        # For log cycling
        self.last_log_cycle = time.monotonic()
        self._ts_base = time.monotonic() 

    def timestamp(self, ts=None):
        '''Make (or convert) a mono timestamp relative to our start time.'''
        if ts is None:
            ts = time.monotonic()
        return ts - self._ts_base

    def nt_init(self):
        '''initialize NetworkTables stuff'''
        nt = self.nt = ntcore.NetworkTableInstance.getDefault()
        nt.setServerTeam(8089)

        chute_mode_topic = nt.getStringTopic(Name.CHUTE_MODE)
        self.chute_mode_sub = chute_mode_topic.subscribe('none')

        tof_mode_topic = nt.getStringTopic(Name.TOF_MODE)
        self.tof_mode_sub = tof_mode_topic.subscribe('none',
           ntcore.PubSubOptions(keepDuplicates=True))

        self.mode_poller = ntcore.NetworkTableListenerPoller(nt)
        self.mode_poller.addListener(self.tof_mode_sub, ntcore.EventFlags.kValueAll)
        self.mode_poller.addListener(self.chute_mode_sub, ntcore.EventFlags.kValueAll)

        self.ts_dist_pub = nt.getDoubleArrayTopic(Name.TS_DIST_MM).publish()
        self.ts_dist_pub.set([0, 0])

        self.dist_pub = nt.getFloatTopic(Name.DIST_MM).publish()
        self.dist_pub.set(0)

        self.corner_pub = nt.getFloatArrayTopic(Name.CORNERS).publish()
        self.corner_pub.set([0.0, 0.0])

        self.corner_ts_pub = nt.getFloatTopic(Name.CORNER_TS).publish()
        self.corner_ts_pub.set(0.0)
        self.corner_dist_pub = nt.getFloatTopic(Name.CORNER_DIST_MM).publish()
        self.corner_dist_pub.set(0.0)
        
        # Match state topics - for future use with match-based log cycling
        self.match_time_sub = nt.getDoubleArrayTopic(Name.MATCH_TIME).subscribe([0, 0])
        self.fms_attached_sub = nt.getBooleanTopic(Name.FMS_ATTACHED).subscribe(False)
        self.enabled_sub = nt.getBooleanTopic(Name.ENABLED).subscribe(False)

        # for debugging, serve our NT instance
        if self.args.serve:
            nt.startServer()
        else:
            nt.startClient4("tof")

        self.tof_mode_pub = tof_mode_topic.publish()
        self.tof_mode_pub.set('none')
        self.chute_mode_pub = chute_mode_topic.publish()
        self.chute_mode_pub.set('home/right')

        # Brief pause, hoping beyond hope that NT will compare clocks and
        # set the offset during this.
        # Note: empirically measured it takes only about 5ms for it to
        # measure the offset after starting the client, so this wait is adequate.
        time.sleep(0.2)

        # Rust will use Instant::now() which is time.monotonic(),
        # while ntcore._now() is supposedly the monotonic clock
        # but has a value like an epoch time, so it appears they initialize
        # it with the offset between epoch time and time.monotonic(),
        # so we need to get that back so we can convert Rust times to
        # ntcore times so that getServerTimeOffset() can be added to
        # convert the times to FPGATimestamp values... sigh.
        # Note: repeated connections to the robot will produce server time
        # offset values that vary up and down within a roughly +/-800us
        # range, so our overall accuracy on any given connection is about
        # that much.  If we don't care about accuracy beyond roughly 1ms
        # then we can probably just live with that, ignoring drift.
        # With fire3 and the one Rio we have roughly 6ppm drift, e.g. we
        # saw about 35ms drift over roughly a two hour period. Also low.
        now = time.monotonic()
        nt_now = ntcore._now()
        self._nt_offset = nt_now / 1e6 - now
        self.log.info('NT time: now=%s, offset=%s, (mono-rel=%s)',
            nt_now, self._nt_offset, self.timestamp(now))

    def run(self):
        self.log.info('-' * 40)
        self.log.info('tof v%s', __version__)
        self.log.info('args %r', self.args)

        def exit_handler(_signal, _frame):
            self.shutdown()
        signal.signal(signal.SIGINT, exit_handler)

        self.nt_init()

        self.cd = CornerDetector(400)
        self.running = True

        self.pins = SetPins([5, 14])

        self.mgr = SensorManager()

        self.running = True
        self.loop()

    def mono_to_fpga(self, ts):
        '''Convert a mono timestamp (including from Rust Instant::now()
        to an FPGA timestamp using the latest server time offset. Note
        that the server time offset currently does NOT get updated
        after the first time so our clocks will drift relative to each other.'''
        # # Until we're connected this will return None so make sure
        # # we don't crash, by just calling that 0...
        # offset = self.nt.getServerTimeOffset() or 0 # convert None to 0

        # # return ts - time.monotonic() + (ntcore._now() + offset) / 1e6
        # # Convert our time.monotonic() timestamps to ntcore._now()'s
        # # view of things by applying the offset, which we believe is
        # # static, and then applying the server time offset to get
        # # this into the FPGATimestamp domain that the robot expects.
        # # Also note that that works in microseconds, not seconds.
        # return int((ts + self._nt_offset) * 1e6 + offset)

        offset_or_none = self.nt.getServerTimeOffset()
        if offset_or_none is None:
            return None
        return ts - time.monotonic() + (ntcore._now() + offset_or_none) / 1e6

    def check_log_cycle(self):
        """Check if it's time to cycle log files based on time"""
        if self.args.log_cycle_time <= 0:
            return  # Log cycling disabled
            
        current_time = time.monotonic()
        elapsed_minutes = (current_time - self.last_log_cycle) / 60
        
        if elapsed_minutes >= self.args.log_cycle_time:
            self.log.info(f"Cycling log file after {elapsed_minutes:.1f} minutes")
            self.log_manager.start_new_log()
            self.last_log_cycle = current_time

    def on_reading(self, ts, dist_mm, status, delta):
        # Check for log cycling
        self.check_log_cycle()
        
        # Log the distance reading
        self.log_manager.append_double(Name.DIST_MM, dist_mm, self.timestamp(ts))
        
        flush = False # whether to flush NT (any time we publish)
        cd = self.cd
        cd.add_record(ts, dist_mm, self.speed)
        if cd.found_corner():
            self.saw_corner = True
            corner_ts_fpga = self.mono_to_fpga(cd.corner_timestamp)
            if corner_ts_fpga is not None:
                # Log corner detection with both timestamps
                # corner_ts_micros = int(cd.corner_timestamp * 1_000_000)
                self.log_manager.append_double(Name.CORNER, cd.corner_dist, cd.corner_timestamp)
                # self.log_manager.append_double(Name.CORNER_DIST_MM, cd.corner_dist, cd.corner_timestamp)
                self.log_manager.append_double(Name.CORNER, 0, ts)
            
                # Convert corner mono time to FPGA for the robot and send via NT
                # corner_ts_fpga = self.mono_to_fpga(cd.corner_timestamp)
                self.corner_pub.set([corner_ts_fpga, cd.corner_dist])
                self.corner_ts_pub.set(cd.corner_timestamp)
                self.corner_dist_pub.set(cd.corner_dist)
                flush = True

                self.log.info("CORNER: @%.3f,%.3f,%.3f,%.3fs", cd.corner_timestamp,
                     ts, corner_ts_fpga, self.speed)

            cd.log_timing()

            cd.reset()
        else:
            # self.log.info("dist,%8.3f,%5.0f,%2d,%5.3f", ts, dist_mm, status, delta)
            if self.args.stdout:
                print("dist,%8.3f,%5.0f,%2d,%5.3f      " % (ts, dist_mm, status, delta), end='\r')

        if self.tof_mode == 'corner':
            ts_fpga = self.mono_to_fpga(ts)
            if ts_fpga is not None:
                # Log to DataLog
                # self.log_manager.append_double_array(Name.TS_DIST_MM, [ts_fpga, dist_mm], ts)
            
                # Publish to NT
                self.ts_dist_pub.set([ts_fpga, dist_mm])

                if self.saw_corner:
                    flush = True

        self.dist_pub.set(dist_mm)
        if flush:
            self.nt.flush()

    # Map of modes to GPIO pin indices [5, 14]
    MODE_MAP = {
        'none': 0, # temporarily map to right so we keep logging something all the time
        'left': 1,
        'right': 0,
        'front-left': 1,
        'front-right': 0,
        'rear-left': 1,
        'rear-right': 0,
    }

    def loop(self):
        # chute_mode = 'home/right'
        chute_side = 'right'
        self.pins.set_index_high(self.MODE_MAP.get(chute_side))

        def reader():
            self.mgr.read(self.args.timing, self.args.inter, callback=self.on_reading)
        threading.Thread(target=reader, daemon=True).start()

        loop_ts = time.monotonic()
        while self.running:
            # poll for robot info... would be more efficient to use an NT listener
            for event in self.mode_poller.readQueue():
                # ts = self.timestamp()
                ts = time.monotonic()
                self.log.debug('event: %s', event)

                topic = event.data.topic.getName()
                if topic == Name.CHUTE_MODE:
                    val = event.data.value.getString()
                    self.log.info('chute: %s', val)
                    
                    # Log to DataLog with monotonic timestamp
                    self.log_manager.append_string(Name.CHUTE_MODE, val, ts)
                    
                    pos, _, side = val.partition('/')
                    if pos != 'load':
                        side = 'left' if side == 'right' else 'right'
                    if chute_side != side:
                        chute_side = side

                        # handle mode changes
                        # disabling will cause any current reading thread to exit
                        self.pins.set_index_high(None)
                        # this gives enough time for the thread to attempt a reading
                        # and get an i2c error because the sensor will be offline
                        time.sleep(0.1)
                        self.pins.set_index_high(self.MODE_MAP.get(chute_side))
                        self.log.info('selected tof: %s (from %s)', chute_side, val)

                        self.cd.reset()

                elif topic == Name.TOF_MODE:
                    self.ts_dist_pub.set([0, 0])
                    self.dist_pub.set(0)
                    val = event.data.value.getString()
                    self.saw_corner = False
                    self.tof_mode = val
                    self.log.info('tof mode: %s', val)
                    
                    # Log to DataLog with monotonic timestamp
                    self.log_manager.append_string(Name.TOF_MODE, val, ts)

            # pause to avoid busy cpu, as we're not yet using full NT listeners
            time.sleep(0.01)

            now = time.monotonic()
            elapsed = now - loop_ts
            if elapsed > 0.075:
                self.log.warning('long loop time %.3fs', elapsed)
            loop_ts = now
            
            # TODO: Future enhancement - implement match-based log cycling
            # This would check match state from NetworkTables and start a new log
            # file at the beginning of each match

    def shutdown(self):
        self.log.warning('shutting down')
        self.running = False
        self.mgr.shutdown()
        self.log_manager.close()
        self.nt.stopClient()
        if self.args.serve:
            self.nt.stopServer()


def main():
    args = get_args()

    filename = LOG_DIR / f"tof-{datetime.datetime.now().strftime('%Y%m%d-%H%M%S.log')}"
    handlers = [logging.FileHandler(filename, encoding="utf-8")]
    if args.stdout:
        handlers.append(logging.StreamHandler(sys.stdout))
    level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=level,
        format="%(asctime)s.%(msecs)03d:%(levelname)5s:%(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=handlers
    )

    logging.getLogger('cd').setLevel(logging.DEBUG if args.debug_cd else logging.INFO)

    tof = TofMain(args)
    try:
        tof.run()
    finally:
        logging.info('main exiting')

if __name__ == "__main__":
    main()
