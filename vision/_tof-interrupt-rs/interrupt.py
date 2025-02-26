import select
import time
from vl53l1x import TofSensor

# Initialize sensor
sensor = TofSensor(1)  # Using I2C bus 1
sensor.init(voltage_2v8=True)

# Configure sensor
sensor.set_timing_budget_ms(50)
sensor.set_inter_measurement_period_ms(55)

# Start ranging with interrupts
sensor.start_ranging()
fd = sensor.start_interrupt_handler("gpiochip0", 17, 100)  # Using GPIO17 for interrupts

# Create polling object for the file descriptor
poller = select.poll()
poller.register(fd, select.POLLIN)

try:
    while True:
        # Wait for interrupt
        events = poller.poll()
        for fd, event in events:
            if event & select.POLLIN:
                # Read the data
                reading = sensor.get_interrupt_reading()
                if reading:
                    timestamp, distance = reading
                    print(f"Time: {timestamp}, Distance: {distance}mm")
except KeyboardInterrupt:
    print("Stopping...")
finally:
    sensor.stop_ranging()
