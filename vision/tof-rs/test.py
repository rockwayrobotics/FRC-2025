from vl53l1x import TofSensor
import time

def main():
    # Create and configure sensor
    sensor = TofSensor(bus=1)
    sensor.init(True)  # Initialize with 2.8V mode

    # Configure sensor parameters
    sensor.set_long_distance_mode(False)  # Short distance mode for faster readings
    sensor.set_timing_budget_ms(20)  # 20ms is minimum for short mode
    sensor.set_inter_measurement_period_ms(25)  # Must be >= timing budget

    # Print configuration
    print("Sensor Configuration:")
    print(f"Timing budget: {sensor.get_timing_budget_ms()} ms")
    print(f"Inter-measurement period: {sensor.get_inter_measurement_period_ms()} ms")

    # Start streaming mode
    print("\nStarting streaming mode...")
    sensor.start_streaming()

    try:
        # Main loop
        while True:
            # Get latest reading (timestamp, distance)
            if reading := sensor.get_latest_reading():
                timestamp, distance = reading
                print(f"Time: {timestamp:8.3f}s | Distance: {distance:4d}mm")
            # time.sleep(0.001)  # Small sleep to prevent CPU spinning

    except KeyboardInterrupt:
        print("\nStopping streaming...")
        sensor.stop_streaming()
        print("Sensor stopped.")

if __name__ == "__main__":
    main()