import csv

class fake_VL53L1x:

    def __init__(self, i2c_bus=1, i2c_address=0x29, tca9548a_num=255, tca9548a_addr=0):
        """Initialize attributes to mimic a real VL53L1X ToF Sensor from ST"""
        self._i2c_bus = i2c_bus
        self.i2c_address = i2c_address
        self._tca9548a_num = tca9548a_num
        self._tca9548a_addr = tca9548a_addr
        self.dev_initialized = False
        self.index = 0
        # Register Address
        self.ADDR_UNIT_ID_HIGH = 0x16  # Serial number high byte
        self.ADDR_UNIT_ID_LOW = 0x17   # Serial number low byte
        self.ADDR_I2C_ID_HIGH = 0x18   # Write serial number high byte for I2C address unlock
        self.ADDR_I2C_ID_LOW = 0x19    # Write serial number low byte for I2C address unlock
        self.ADDR_I2C_SEC_ADDR = 0x8a  # Write new I2C address after unlock

    def open(self, reset=False):
        self.dev_initialized = True

    def fake_init(self, tof_num, filepath):
        self.tof_num = tof_num
        self.filepath = filepath
        with open(filepath, mode ='r') as file:
            lines = csv.DictReader(file)
            self.data = list(lines)
        self.index = 0

    def close(self):
        self.dev_initialized = False
        
    def set_user_roi(self, user_roi):
        self.roi = user_roi

    def start_ranging(self, mode=None):
        """Start fake VL53L1X ToF Sensor Ranging"""
        self.ranging = True

    def set_distance_mode(self, mode):
        """Set distance mode

        :param mode: One of 1 = Short, 2 = Medium or 3 = Long

        """
        self.mode = mode

    def stop_ranging(self):
        """Stop fake VL53L1X ToF Sensor Ranging"""
        self.ranging = False

    def get_distance(self):
        """Get next reading from the file data"""
        ts = float(self.data[self.index]['Timestamp'])
        dist = float(self.data[self.index]['/RealOutputs/ToF/Distance/' + str(self.tof_num - 1)])
        self.index = self.index + 1
        return (ts, dist)

    def set_timing(self, timing_budget, inter_measurement_period):
        """Set the timing budget and inter measurement period.

        A higher timing budget results in greater measurement accuracy,
        but also a higher power consumption.

        The inter measurement period must be >= the timing budget, otherwise
        it will be double the expected value.

        :param timing_budget: Timing budget in microseconds
        :param inter_measurement_period: Inter Measurement Period in milliseconds

        """
        # do we need to check to ensure that this is correct?
        self.timing_budget = timing_budget
        self.inter_measurement_period = inter_measurement_period
        
    def set_timing_budget(self, timing_budget):
        """Set the timing budget in microseocnds"""
        self.timing_budget = timing_budget

    def set_inter_measurement_period(self, period):
        """Set the inter-measurement period in milliseconds"""
        self.inter_measurement_period = period

    def change_address(self, new_address):
        self.i2c_address = new_address
        return True