from vl53l1x import TofSensor
import time

def setup(sensor):
    sensor.init(True)  # Initialize with 2.8V mode

    # Configure sensor parameters
    sensor.set_long_distance_mode(False)  # Short distance mode for faster readings
    sensor.set_timing_budget_ms(20)  # 20ms is minimum for short mode
    sensor.set_inter_measurement_period_ms(25)  # Must be >= timing budget

    sensor.start_streaming()


def main():
    # Create and configure sensor
    initialized = False
    sensor = TofSensor(bus=1)

    try:
        # Main loop
        print("running")
        last = 0
        while True:
            if not initialized:
                try:
                    setup(sensor)
                except:
                    time.sleep(0.1)
                    continue
                initialized = True

                print("sensor found, initialized")

                # print("getting reading")
            reading = sensor.get_latest_reading()

            if reading is None:
                initialized = False
                print(f"failed to read")
                continue

            # print("got reading")
            timestamp, distance, valid = reading
            print(timestamp, distance, valid)
            if valid:
                if timestamp != last:
                    print(f"Delta: {timestamp - last} | Time: {timestamp:8.4f}ms | Distance: {distance:4d}mm")
                    last = timestamp
            time.sleep(0.01)  # Small sleep to prevent CPU spinning

    except KeyboardInterrupt:
        print("\nStopping streaming...")
        sensor.stop_streaming()
        print("Sensor stopped.")

if __name__ == "__main__":
    main()
