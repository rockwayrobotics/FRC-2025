#!/usr/bin/env python3

import datetime
import functools
import logging

# import os
from enum import IntEnum
import time
import sys
import signal
import typing

from ntcore import DoubleArraySubscriber, FloatArrayPublisher, NetworkTableInstance, PubSubOptions
from corner_detector import CornerDetector

try:
    import vl53l1x
except ImportError:
    pass
    # FIXME: Need to adapt fake_VL53L1x to work with the new API
    # import fake_VL53L1x as VL53L1X

UPDATE_TIME_MICROS = 20000
INTER_MEASUREMENT_PERIOD_MILLIS = 25

logger = None
running = False
sensor_manager = None


class SensorMode(IntEnum):
    NONE = 0
    FRONT_LEFT = 1
    FRONT_RIGHT = 2
    BACK_LEFT = 3
    BACK_RIGHT = 4

def exit_handler(signal, frame):
    global running
    running = False
    sensor_manager.cleanup()
    sys.exit(0)


signal.signal(signal.SIGINT, exit_handler)


def logging_init(args):
    global logger
    logger = logging.getLogger("tof")
    filename = f"tof-log-{datetime.datetime.now().strftime('%Y%m%d-%H%M%S.log')}"
    handlers = [logging.FileHandler(filename, encoding="utf-8")]
    if args.stdout:
        handlers.append(logging.StreamHandler(sys.stdout))
    logging.basicConfig(level=logging.DEBUG, handlers=handlers)
    logger.info("Start")
    logger.info(f"args={args!r}")


class FakeToFSensor:
    def __init__(self, args, address):
        self.address = address
        self.start_ranging()

    def start_ranging(self):
        self.ranging = True

    def stop_ranging(self):
        self.ranging = False

    def read(self) -> float:
        return 0.0


class ToFSensor:
    """
    Open and start the VL53L1X ranging sensor
    """

    def __init__(self, args, address):
        self.address = address
        self.tof = vl53l1x.TofSensor(bus=1, address=address)
        self.tof.init(True)  # True means set for 2.8V

        assert args.mode in (1, 3), "invalid modes with ULD driver"
        self.tof.set_long_distance_mode(args.mode == 3)

        # roi = vl53l1x.VL53L1xUserRoi(*args.roi) # default wide angle
        # tof.set_user_roi(roi)

        # Lower timing budgets allow for faster updates, but sacrifice accuracy
        self.tof.set_timing_budget_ms(args.timingMS)
        self.tof.set_inter_measurement_period_ms(args.interMS)
        self.start_ranging()

    def start_ranging(self):
        try:
            # Start ranging
            self.tof.start_ranging()
        except:
            try:
                self.tof.stop_ranging()
                self.tof.start_ranging()
            except Exception as e:
                logger.error(f"Failed to start ranging on sensor 0x{self.address:02x}: {e}")
                print(f"Warning: Failed to start ranging on sensor 0x{self.address:02x}")
                self.ranging = False
                raise e
        self.ranging = True

    def stop_ranging(self):
        try:
            self.tof.stop_ranging()
        except:
            pass
        self.ranging = False

    def read(self) -> float:
        while not self.tof.is_data_ready():
            time.sleep(0.001)
        res = self.tof.get_distance()
        self.tof.clear_interrupt()
        return res


class SensorManager:
    def __init__(self):
        self.sensors: typing.Dict[SensorMode, ToFSensor | FakeToFSensor] = {}

    def add_tof(self, args, address: int, location: SensorMode):
        try:
            sensor = ToFSensor(args, address)
        except Exception as e:
            print(f"Warning: Failed to initialize on sensor 0x{address:02x}")
            logger.error(f"Failed to initialize sensor 0x{address:02x}, using fake sensor: {e}")
            sensor = FakeToFSensor(args, address)

        # add sensor to list of active sensors - dict of sensor positions?
        self.sensors[location] = sensor

    def add_sensors(self, args):
        if args.frontLeft > 0:
            self.add_tof(args, args.frontLeft, SensorMode.FRONT_LEFT)
        if args.frontRight > 0:
            self.add_tof(args, args.frontRight, SensorMode.FRONT_RIGHT)
        if args.backLeft > 0:
            self.add_tof(args, args.backLeft, SensorMode.BACK_LEFT)
        if args.backRight > 0:
            self.add_tof(args, args.backRight, SensorMode.BACK_RIGHT)

    def read_sensor(self, location: SensorMode):
        if location in self.sensors.keys():
            return self.sensors[location].read()
        else:
            print(f"Warning: Sensor {location} not found", self.sensors.keys())
            return None

    def cleanup(self):
        # stop ranging on all active sensors
        for sensor in self.sensors.values():
            sensor.stop_ranging()


class FakeSubscriber:
    def __init__(self, values):
        self.values = values

    def get(self):
        return self.values


def nt_init() -> tuple[NetworkTableInstance, DoubleArraySubscriber, FloatArrayPublisher]:
    nt = NetworkTableInstance.getDefault()
    nt.setServerTeam(8089)
    nt.startClient4("tof")
    sensorModeSubscriber = nt.getDoubleArrayTopic("/Pi/SensorMode").subscribe(
        [float(SensorMode.NONE), 0.], PubSubOptions()
    )
    cornerPublisher = nt.getFloatArrayTopic("/Pi/Corners").publish(PubSubOptions())
    return (nt, sensorModeSubscriber, cornerPublisher)


def main(args):
    global running
    global sensor_manager

    logging_init(args)
    nt, sensorModeSubscriber, cornerPublisher = nt_init()
    sensor_manager = SensorManager()
    sensor_manager.add_sensors(args)

    detector = CornerDetector(400)
    running = True
    mode = SensorMode.NONE
    if args.debugMode:
        mode = SensorMode.FRONT_LEFT
        sensorModeSubscriber = FakeSubscriber([float(SensorMode.FRONT_LEFT), 0.45])

    lastMode = mode
    while running:
        modeFloat, speed = sensorModeSubscriber.get()
        mode = int(modeFloat)
        if mode < SensorMode.NONE or mode > SensorMode.BACK_RIGHT:
            logger.warning(f"Invalid sensor mode {mode}")
            mode = SensorMode.NONE

        if mode == SensorMode.NONE:
            # FIXME: Consider logging sensor outputs even when not reporting them
            continue

        if lastMode != mode:
            detector.reset()
            lastMode = mode

        distance_in_mm = sensor_manager.read_sensor(mode)
        if distance_in_mm is None:
            continue

        timestamp = time.monotonic()
        detector.add_record(timestamp, distance_in_mm, speed)
        if detector.found_corner():
            cornerPublisher.set([detector.corner_timestamp, detector.corner_angle])
            nt.flush()
            logger.info("CORNER: %.3f,%.3f", timestamp, detector.corner_timestamp)
            detector.reset()
        else:
            logger.info("DISTANCE: %.3f,%.3f", timestamp, distance_in_mm)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    # Default address is 0x29. With the address translator, we have these options:
    # 0x29 ^ 0x40 = 0x69 = 105 (A4 and A5 on)
    # 0x29 ^ 0x50 = 0x79 = 121 (A5 off, A4 on)
    # 0x29 ^ 0x60 = 0x49 = 73 (A5 off, A4 on)
    # 0x29 ^ 0x70 = 0x59 = 89 (A4 and A5 off)
    # Defaults to using front left on the default address if none of these are set
    parser.add_argument(
        "-1",
        "--frontLeft",
        "--frontleft",
        type=functools.wraps(int)(lambda x: int(x, 0)),
        default=0x29,
    )
    parser.add_argument(
        "-2",
        "--frontRight",
        "--frontright",
        type=functools.wraps(int)(lambda x: int(x, 0)),
        default=0,
    )
    parser.add_argument(
        "-3",
        "--backLeft",
        "--backleft",
        type=functools.wraps(int)(lambda x: int(x, 0)),
        default=0,
    )
    parser.add_argument(
        "-4",
        "--backRight",
        "--backright",
        type=functools.wraps(int)(lambda x: int(x, 0)),
        default=0,
    )
    parser.add_argument("--debugMode", action="store_true")
    parser.add_argument("--roi", default="0,15,15,0")
    parser.add_argument(
        "-t", "--timingMS", type=int, default=int(UPDATE_TIME_MICROS / 1000)
    )
    parser.add_argument(
        "-i", "--interMS", type=int, default=INTER_MEASUREMENT_PERIOD_MILLIS
    )
    parser.add_argument("-f", "--fake")
    parser.add_argument("-m", "--mode", type=int, default=1)
    parser.add_argument("--stdout", action="store_true")

    args = parser.parse_args()
    args.roi = tuple(int(x) for x in args.roi.split(","))
    # Region of interest is top-left X, top-left Y, bottom-right X, bottom-right Y.
    # Minimum size is 4x4, values must be within 0-15 (inclusive)
    if abs(args.roi[0] - args.roi[2]) < 3 or abs(args.roi[1] - args.roi[3]) < 3:
        print("ROI must be at least width and height 4")
        sys.exit(0)
    
    # Store hex-printable values for logging
    args.frontLeftHex = hex(args.frontLeft)
    args.frontRightHex = hex(args.frontRight)
    args.backLeftHex = hex(args.backLeft)
    args.backRightHex = hex(args.backRight)

    if args.timingMS + 4 > args.interMS:
        args.interMS = args.timingMS + 4
        print(
            f"Inter-measurement must be greater than timing budget, setting inter-measurement period to {args.interMS}"
        )

    main(args)
