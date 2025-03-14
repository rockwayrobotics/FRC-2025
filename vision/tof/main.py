#!/usr/bin/env python3

import datetime
import logging
import math
from pathlib import Path
import sys
import signal
import threading
import time

import ntcore

try:
    from vl53l1x import TofSensor, ThreadError, SetPins
except ImportError:
    print('mock sensor support not supported at the moment')
    raise
    # FIXME: Need to adapt fake_VL53L1x to work with the new API
    # import fake_VL53L1x as VL53L1X

from .corner_detector import CornerDetector

__version__ = "0.1"

LOG_DIR = Path("logs")

DEFAULT_TOF_ADDRESS = 0x29
DEFAULT_TIMING = 20
DEFAULT_INTER = 25



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
                            self.log.error(f"Error: {ex}")

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
                        self.log.warn("DEACTIVATED")
                        continue

                except Exception as ex:
                    if not warned:
                        warned = True
                        self.log.error(f"Error: {ex}")

                    running = False
                    continue

                else:
                    ts, distance, status = reading

                    delta = ts - ts_last
                    ts_last = ts

                    if callback is not None:
                        callback(ts, distance, status, delta)

                    time.sleep(0.01)  # Small sleep to prevent CPU spinning
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
    # parser.add_argument("-f", "--fake")
    # parser.add_argument("-m", "--mode", type=int, default=1)
    parser.add_argument("--serve", action="store_true",
        help="serve NT instance (for testing)")
    parser.add_argument("--stdout", action="store_true")
    parser.add_argument("--slope", type=float, default=400,
        help="slope threshold (for corner detector) (default %(default)s)")
    parser.add_argument("--speed", type=float, default=0.45,
        help="fake speed (for testing, default %(default)s)")

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

    def nt_init(self):
        '''initialize NetworkTables stuff'''
        nt = self.nt = ntcore.NetworkTableInstance.getDefault()
        nt.setServerTeam(8089)

        chute_mode_topic = nt.getStringTopic("/Pi/chute_mode")
        self.chute_mode_sub = chute_mode_topic.subscribe('none',
           ntcore.PubSubOptions(keepDuplicates=True))

        tof_mode_topic = nt.getStringTopic("/Pi/tof_mode")
        self.tof_mode_sub = tof_mode_topic.subscribe('none',
           ntcore.PubSubOptions(keepDuplicates=True))

        self.mode_poller = ntcore.NetworkTableListenerPoller(nt)
        self.mode_poller.addListener(self.tof_mode_sub, ntcore.EventFlags.kValueAll)
        self.mode_poller.addListener(self.chute_mode_sub, ntcore.EventFlags.kValueAll)

        # speedTopic = nt.getDoubleTopic("/Pi/speed")
        # self.speedSub = speedTopic.subscribe(0.0)

        self.ts_dist_pub = nt.getDoubleArrayTopic("/Pi/ts_dist_mm").publish()
        self.ts_dist_pub.set([0, 0])

        self.dist_pub = nt.getFloatTopic("/Pi/dist_mm").publish()
        self.dist_pub.set(0)

        self.corner_pub = nt.getFloatArrayTopic("/Pi/Corners").publish()
        self.corner_pub.set([0.0, 0.0])

        self.corner_ts_pub = nt.getFloatTopic("/Pi/corner_ts").publish()
        self.corner_ts_pub.set(0.0)

        # for debugging, serve our NT instance
        if self.args.serve:
            nt.startServer()
            # self.sensorModePub = sensorModeTopic.publish()
            # self.sensorModePub.set(0)
            # self.speedPub = speedTopic.publish()
            # self.speedPub.set(0.0)
        else:
            nt.startClient4("tof")

        self.tof_mode_pub = tof_mode_topic.publish()
        self.tof_mode_pub.set('none')
        self.chute_mode_pub = chute_mode_topic.publish()
        self.chute_mode_pub.set('home/right')

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


    def on_reading(self, ts, dist_mm, status, delta):
        flush = False # whether to flush NT (any time we publish)
        cd = self.cd
        nt_time_offset = self.nt.getServerTimeOffset() / 1e6
        cd.add_record(ts, dist_mm, self.speed)
        if cd.found_corner():
            self.saw_corner = True
            corner_ts = cd.corner_timestamp + nt_time_offset
            self.corner_pub.set([corner_ts, cd.corner_angle])
            self.corner_ts_pub.set(cd.corner_timestamp)
            flush = True

            if self.args.stdout:
                print()
            self.log.info("CORNER: %.3f,%.3f,%.3fs", ts, corner_ts, self.speed)
            cd.log_timing()

            cd.reset()
        else:
            self.log.info("dist,%8.3f,%5.0f,%2d,%5.3f", ts, dist_mm, status, delta)
            if self.args.stdout:
                print("dist,%8.3f,%5.0f,%2d,%5.3f      " % (ts, dist_mm, status, delta), end='\r')

        if self.tof_mode == 'corner':
            self.ts_dist_pub.set([ts + nt_time_offset, dist_mm])
            self.dist_pub.set(dist_mm)
            if self.saw_corner:
                flush = True

        if flush:
            self.nt.flush()
    

    # Map of modes to GPIO pin indices [5, 14]
    MODE_MAP = {
        'none': None,
        'left': 1,
        'right': 0,
        'front-left': 1,
        'front-right': 0,
        'rear-left': 1,
        'rear-right': 0,
    }

    def loop(self):
        chute_mode = 'right'
        self.pins.set_index_high(self.MODE_MAP.get(chute_mode))

        def reader():
            self.mgr.read(self.args.timing, self.args.inter, callback=self.on_reading)
        thread = threading.Thread(target=reader).start()

        loop_ts = time.monotonic()
        while self.running:
            # poll for robot info... would be more efficient to use an NT listener
            for event in self.mode_poller.readQueue():
                self.log.debug('event: %s', event)

                topic = event.data.topic.getName()
                if topic == '/Pi/chute_mode':
                    val = event.data.value.getString()
                    self.log.info('chute: %s', val)
                    pos, _, side = val.partition('/')
                    if chute_mode != side:
                        chute_mode = side

                        # handle mode changes
                        # disabling will cause any current reading thread to exit
                        self.pins.set_index_high(None)
                        # this gives enough time for the thread to attempt a reading
                        # and get an i2c error because the sensor will be offline
                        time.sleep(0.1)
                        self.pins.set_index_high(self.MODE_MAP.get(chute_mode))
                        self.log.info('selected tof: %s', chute_mode)

                        self.cd.reset()

                elif topic == '/Pi/tof_mode':
                    self.ts_dist_pub.set([0, 0])
                    self.dist_pub.set(0)
                    val = event.data.value.getString()
                    self.saw_corner = False
                    self.tof_mode = val
                    self.log.info('tof mode: %s', val)

            # pause to avoid busy cpu, as we're not yet using full NT listeners
            time.sleep(0.05)
            # self.log.debug('loop')

            now = time.monotonic()
            elapsed = now - loop_ts
            if elapsed > 0.075:
                self.log.warn('long loop time %.3fs', elapsed)
            loop_ts = now
            

    def shutdown(self):
        self.log.warn('shutting down')
        self.running = False
        self.mgr.shutdown()
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
