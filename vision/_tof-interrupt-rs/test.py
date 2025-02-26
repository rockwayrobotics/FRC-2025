from vl53l1x import PyVL53L1X

import time

# Create sensor with custom timing parameters
sensor = PyVL53L1X(bus=1)
# Initialize
sensor.init(True)
sensor.set_long_distance_mode(False)
sensor.set_timing_budget_ms(20)
sensor.set_inter_measurement_period_ms(25)

# time.sleep(0.1)
# Read back the actual values
actual_budget = sensor.get_timing_budget_ms()
actual_period = sensor.get_inter_measurement_period_ms()
print(f"Timing budget: {actual_budget} ms")
print(f"Inter-measurement period: {actual_period} ms")

# Try changing the values

# Verify the new values
# print(f"New timing budget: {sensor.get_timing_budget_ms()} ms")
# print(f"New inter-measurement period: {sensor.get_inter_measurement_period_ms()} ms")

# data = sensor.read_bytes(0, 256)
# for index, value in enumerate(data):
#     print(f"0x{index:02x}: 0x{value:02x}")

# Start ranging
sensor.start_ranging()

last = time.monotonic()
try:
    while True:
        if sensor.is_data_ready():
            now = time.monotonic()
            distance = sensor.get_distance()
            print(f"{now - last} | Distance: {distance}mm")
            sensor.clear_interrupt()
            last = now
except KeyboardInterrupt:
    sensor.stop_ranging()
