#!/usr/bin/env python3

import datetime
from enum import IntEnum
import logging
from pathlib import Path
import sys
import signal
import threading
import time

from ntcore import (NetworkTableInstance, PubSubOptions,
    NetworkTableListenerPoller, EventFlags)
from corner_detector import CornerDetector

try:
    from vl53l1x import TofSensor, ThreadError, SetPins
except ImportError:
    raise
    # FIXME: Need to adapt fake_VL53L1x to work with the new API
    # import fake_VL53L1x as VL53L1X

__version__ = "0.1"

LOG_DIR = Path("logs")

DEFAULT_TOF_ADDRESS = 0x29
DEFAULT_TIMING = 20
DEFAULT_INTER = 25

class SensorMode(IntEnum):
    NONE = 0
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4


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
        self.tof.init(True)  # True means set for 2.8V

        self.tof.set_long_distance_mode(false)  # short distance mode for faster readings

        # Lower timing budgets allow for faster updates, but sacrifice accuracy
        self.tof.set_timing_budget_ms(timing)

        # inter-measurement period must be higher than timing budget so use max()
        self.tof.set_inter_measurement_period_ms(max(timing, inter))

        self.tof.start_streaming()


    def is_running(self):
        return self.tof.is_running()


    def read(self, timing, inter, callback=None):
        self.enabled = True
        try:
            ts_last = time.monotonic() # try to match the one Rust uses...
            running = False
            while self.enabled:
                if not running:
                    # Try reconfiguring sensor.  If it's present it will
                    # end up configured properly and streaming.  If it's
                    # not present or there's a problem, we'll get an
                    # exception, then try again after a brief pause.
                    try:
                        self.configure(timing, inter)
                    except Exception:
                        time.sleep(0.1)
                        continue
                    running = True

                    self.log.debug("active")

                try:
                    # This should block until there's a reading or error.
                    reading = self.tof.get_reading()

                    if reading is None:
                        running = False
                        self.log.warn("thread ended")
                        continue

                except Exception as ex:
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


    def nt_init(self):
        nt = self.nt = NetworkTableInstance.getDefault()
        nt.setServerTeam(8089)

        # for debugging, serve our NT instance
        if self.args.serve:
            nt.startServer()

        nt.startClient4("tof")

        self.sensorModeSub = nt.getIntegerTopic("/Pi/SensorMode").subscribe(
            SensorMode.NONE, PubSubOptions(keepDuplicates=True))
        self.speedSub = nt.getDoubleTopic("/Pi/Speed").subscribe(0.0, PubSubOptions())

        self.cornerPub = nt.getFloatArrayTopic("/Pi/Corners").publish(PubSubOptions())
        self.modeListener = NetworkTableListenerPoller(nt)
        self.modeListener.addListener(self.sensorModeSub, EventFlags.kValueAll)

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
        # if self.args.debug:
        #     mode = SensorMode.FRONT_LEFT
        #     sensorModeSubscriber = FakeSubscriber([float(SensorMode.FRONT_LEFT), 0.45])

        self.pins = SetPins([5, 14])

        self.mgr = SensorManager()

        self.running = True
        self.loop()


    def on_reading(self, ts, dist, status, delta):
        cd = self.cd
        cd.add_record(ts, dist_mm, self.speed)
        if cd.found_corner():
            self.cornerPub.set([cd.corner_timestamp, cd.corner_angle])
            self.nt.flush()
            self.log.info("CORNER: %.3f,%.3f", ts, cd.corner_timestamp)
            cd.reset()
        else:
            self.log.info("DIST: %8.3f,%5.0f,%2d,%5.3f", ts, dist_mm, status, delta)
    

    # Map of modes to GPIO pin indices [5, 14]
    MODE_MAP = {
        SensorMode.NONE: None,
        SensorMode.FRONT_LEFT: 1,
        SensorMode.FRONT_RIGHT: 0,
        SensorMode.BACK_LEFT: 1,
        SensorMode.BACK_RIGHT: 0,
    }

    def loop(self):
        lastMode = SensorMode.NONE

        def reader():
            self.log.info('run reader thread')
            try:
                self.mgr.read(self.args.timing, self.args.inter, self.on_reading)
            finally:
                self.log.info('exit reader thread')

        thread = None
        while self.running:
            # poll for robot info... would be more efficient to use an NT listener
            event = None
            for event in self.modeListener.readQueue():
                self.log.debug('event: %s', event)
            # keep only the last one
            if event is not None:
                mode = event.data.value.value()

                self.speed = self.speedSub.get()
                self.log.info('mode %s, speed %s', mode, self.speed)

                # handle mode changes
                if mode != lastMode:
                    # disabling will cause any current reading thread to exit
                    self.pins.set_index_high(None)
                    # this gives enough time for the thread to attempt a reading
                    # and get an i2c error because the sensor will be offline
                    time.sleep(0.1)
                    index = self.MODE_MAP.get(mode) 
                    self.pins.set_index_high(index)
                    self.log.info('selected side %s', index)
                    lastMode = mode

                self.cd.reset()
                self.mgr.shutdown()
                thread = threading.Thread(target=reader)

            # pause to avoid busy cpu, as we're not yet using full NT listeners
            time.sleep(0.5)
            # self.log.debug('loop')

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
    logging.basicConfig(level=level, handlers=handlers)

    tof = TofMain(args)
    try:
        tof.run()
    finally:
        logging.info('main exiting')

if __name__ == "__main__":
    main()
