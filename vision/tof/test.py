#!/usr/bin/env python3
'''Test script for vl53l1x package.'''

# pylint: disable=missing-function-docstring,no-name-in-module
# At least temporarily don't whine about "except Exception" as we haven't
# stabilized the set of exceptions we'll actually be using.
# pylint: disable=broad-exception-caught

import time

from vl53l1x import TofSensor #, ThreadError, SetPins

def setup(sensor):
    sensor.init(True)  # Initialize with 2.8V mode

    # Configure sensor parameters
    sensor.set_long_distance_mode(False)  # Short distance mode for faster readings
    sensor.set_timing_budget_ms(20)  # 20ms is minimum for short mode
    sensor.set_inter_measurement_period_ms(25)  # Must be >= timing budget

    sensor.start_streaming()


def main():
    # This will raise OSError if there's no /dev/i2c-1.
    sensor = TofSensor(bus=1)

    ts_last = 0 # remember previous timestamp in order to show delta

    running = False
    try:
        # Main loop
        print("running")
        while True:
            if not running:
                try:
                    setup(sensor)
                except Exception:
                    time.sleep(0.1)
                    continue
                running = True

                print("sensor enabled")

            try:
                # This should block until there's a reading or error.
                reading = sensor.get_reading()

                if reading is None:
                    running = False
                    print("thread ended")
                    continue

            except Exception as ex:
                print(f"Error: {ex}")
                running = False
                continue

            # print("got reading")
            timestamp, distance, status = reading
            # print(timestamp, distance, status)
            # print(f"Time: {timestamp:8.4f}ms | Dist: {distance:4d}mm"
            #     f" Delta: {timestamp - ts_last}")
            ts_last = timestamp

            # time.sleep(0.01)  # Small sleep to prevent CPU spinning

    except KeyboardInterrupt:
        print("\nStopping streaming...")
        sensor.stop_streaming()
        print("Sensor stopped.")

if __name__ == "__main__":
    main()
