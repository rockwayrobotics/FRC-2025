import vl53l1x

# Create a PinSetup instance with sensor option 1
# This will set GPIO pin 5 to output mode and drive it low
pin_setup = vl53l1x.PinSetup(1)
print(f"Current sensor option: {pin_setup.get_sensor_option}")

# Create a ToF sensor instance
tof = vl53l1x.TofSensor(1)  # Using i2c bus 1
tof.init(voltage_2v8=True)
tof.set_timing_budget_ms(33)
tof.set_inter_measurement_period_ms(50)
tof.start_ranging()

# Get a distance reading
if tof.is_data_ready():
    distance = tof.get_distance()
    print(f"Distance: {distance} mm")

# Stop ranging when done
tof.stop_ranging()