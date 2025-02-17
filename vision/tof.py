#!/usr/bin/env python

import os
import time
import sys
import signal

import corner_detector as cd
try:
  import VL53L1X
except ImportError:
  import fake_VL53L1x as VL53L1X

UPDATE_TIME_MICROS = 66000
INTER_MEASUREMENT_PERIOD_MILLIS = 70

print("""tof.py

Log time and distance measurements for two sensors to stdout.

Press Ctrl+C to exit.

""")

def exit_handler(signal, frame):
    global running
    global tof
    global tof2
    running = False
    if tof is not None:
        tof.stop_ranging()
    if tof2 is not None:
        tof2.stop_ranging()
    sys.exit(0)

signal.signal(signal.SIGINT, exit_handler)

def tof_init(args, address):
    """
    Open and start the VL53L1X ranging sensor
    """
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=address)
    tof.open()  # Initialise the i2c bus and configure the sensor

    tof.set_distance_mode(1)

    roi = VL53L1X.VL53L1xUserRoi(*args.roi) # default wide angle

    tof.set_user_roi(roi)

    # Lower timing budgets allow for faster updates, but sacrifice accuracy
    tof.set_timing(args.timingMS * 1000, args.interMS)

    # Start ranging, mode 0 to leave timing unchanged
    tof.start_ranging(1)
    return tof

def get_time_and_distance(tof):
    global in_test_mode
    if in_test_mode:
        # test mode returns both time and distance
        return tof.get_distance()
    else:
        return (time.monotonic(), tof.get_distance())

def main(args):
    global running
    global tof
    global tof2
    global in_test_mode

    tof = tof_init(args, args.address)
    tof2 = tof_init(args, args.address2)

    if args.fake is not None:
        in_test_mode = True
        tof.fake_init(1, args.fake)
        tof2.fake_init(2, args.fake)
    else:
        in_test_mode = False

    detector = cd.CornerDetector(160)
    running = True
    start = time.monotonic()
    while running:
        (time1, distance_in_mm) = get_time_and_distance(tof)  # Grab the range in mm
        (time2, distance_in_mm2) = get_time_and_distance(tof2)  # Grab the range in mm
        if distance_in_mm <= 0:
            distance_in_mm = 0
        if distance_in_mm2 <= 0:
            distance_in_mm2 = 0
        detector.add_record(time1, distance_in_mm)
        print(f'{time1},{distance_in_mm},{time2},{distance_in_mm2}', flush=True)
        if detector.found_corner():
          print(f'*** Found corner: {detector.corner_timestamp} ***')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    # Default address is 0x29. With the address translator, we have these options:
    # 0x29 ^ 0x40 = 0x69 = 105 (A4 and A5 on)
    # 0x29 ^ 0x50 = 0x79 = 121 (A5 off, A4 on)
    # 0x29 ^ 0x60 = 0x49 = 73 (A5 off, A4 on)
    # 0x29 ^ 0x70 = 0x59 = 89 (A4 and A5 off)
    parser.add_argument('-a', '--address', type=int, default=0x29)
    parser.add_argument('-a2', '--address2', type=int, default=0x69)
    parser.add_argument('--roi', default='0,15,15,0')
    parser.add_argument('-t', '--timingMS', type=int, default=int(UPDATE_TIME_MICROS / 1000))
    parser.add_argument('-i', '--interMS', type=int, default=INTER_MEASUREMENT_PERIOD_MILLIS)
    parser.add_argument('-f', '--fake')

    args = parser.parse_args()
    args.roi = tuple(int(x) for x in args.roi.split(','))
    # Region of interest is top-left X, top-left Y, bottom-right X, bottom-right Y.
    # Minimum size is 4x4, values must be within 0-15 (inclusive)
    if abs(args.roi[0] - args.roi[2]) < 3 or abs(args.roi[1] - args.roi[3]) < 3:
        print('ROI must be at least width and height 4')
        sys.exit(0)

    if args.timingMS + 4 > args.interMS:
        args.interMS = args.timingMS + 4
        print(f'Inter-measurement must be greater than timing budget, setting inter-measurement period to {args.interMS}')

    if args.address == args.address2:
        print('Must choose different addresses')
        sys.exit()

    main(args)
