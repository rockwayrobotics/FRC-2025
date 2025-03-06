#!/usr/bin/env python

import os
import math
import time
import sys
import signal

import VL53L1X

UPDATE_TIME_MICROS = 66000
INTER_MEASUREMENT_PERIOD_MILLIS = 70

print("""log.py

Log time and distance measurements to stdout.

Press Ctrl+C to exit.

""")

def exit_handler(signal, frame):
    global running
    global tof
    global vals
    running = False
    tof.stop_ranging()

    total = 0
    for record in vals:
        total += record[1]
    mean = total / len(vals)

    variance = sum(pow(record[1]-mean,2) for record in vals) / len(vals)  # variance
    stddev = math.sqrt(variance)  # standard deviation
    # for record in vals:
        # print(f'{record[0]},{record[1]}')
    print(f'Done, mean={mean}, stddev={stddev}')
    sys.exit(0)


def main(args):
    global running
    global tof
    global vals
    signal.signal(signal.SIGINT, exit_handler)

    """
    Open and start the VL53L1X ranging sensor
    """
    tof = VL53L1X.VL53L1X(i2c_bus=1, i2c_address=args.address)

    tof.open()  # Initialise the i2c bus and configure the sensor

    tof.set_distance_mode(1)

    roi = VL53L1X.VL53L1xUserRoi(*args.roi) # default wide angle

    tof.set_user_roi(roi)

    # Lower timing budgets allow for faster updates, but sacrifice accuracy
    tof.set_timing(args.timingMS * 1000, args.interMS)

    # Start ranging, mode 0 to leave timing unchanged
    tof.start_ranging(1)

    running = True
    start = time.monotonic()
    vals = []
    while running:
        distance_in_mm = tof.get_distance()  # Grab the range in mm
        if (distance_in_mm > 0):
            vals.append((time.monotonic() - start, distance_in_mm))
            print(f'{time.monotonic() - start},{distance_in_mm}')
            # Sleep is not necessary because Python library waits for readiness
            # time.sleep(args.interMS / 1000.0)
        else:
            print(f'Error returned {distance_in_mm}')

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-a', '--address', default=0x29)
    parser.add_argument('--roi', default='0,15,15,0')
    parser.add_argument('-t', '--timingMS', type=int, default=int(UPDATE_TIME_MICROS / 1000))
    parser.add_argument('-i', '--interMS', type=int, default=INTER_MEASUREMENT_PERIOD_MILLIS)

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

    main(args)