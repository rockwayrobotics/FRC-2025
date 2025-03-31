import random
import re
import time
import logging

import numpy as np

class TofSensor:
    def __init__(self, *_args, **_kwargs):
        self._sim = self._sim_distances()

    def _sim_time(self):
        return time.monotonic()

    def _sim_distances(self):
        base_value = 1050
        noise_range = 10
        ramp_min = 6
        ramp_max = 25
        ramp_probability = 0.015 # 0.015 is one roughly every ~5s
        min_target = 200
        max_target = 500
        noise = lambda n: np.random.uniform(-noise_range/2, noise_range/2, n)

        while True:
            # Generate normal values until we decide to ramp
            while random.random() >= ramp_probability:
                yield base_value + random.uniform(-noise_range, noise_range)
        
            # Decide on a target value for this ramp
            target = random.randint(min_target, max_target)
        
            # Calculate number of steps for down and up ramps
            steps_down = int((base_value - target) / random.randrange(ramp_min, ramp_max))
            steps_up = int((base_value - target) / random.randrange(ramp_min, ramp_max))
        
            # Generate ramp down sequence with noise
            ramp_down = np.linspace(base_value, target, steps_down) + noise(steps_down)
        
            # Generate ramp up sequence with noise
            ramp_up = np.linspace(target, base_value, steps_up) + noise(steps_up)
        
            yield from ramp_down
            yield target
            yield from ramp_up

    def init(self, *_args): pass
    def set_long_distance_mode(self, *_args): pass
    def set_timing_budget_ms(self, *_args): pass
    def set_inter_measurement_period_ms(self, *_args): pass
    def start_streaming(self, *_args): pass
    def stop_streaming(self, *_args): pass
    def is_running(self): return True
    def get_reading(self):
        time.sleep(0.0235)
        return [round(self._sim_time(), 3), int(next(self._sim)), 0]

class SetPins: 
    def __init__(self, *_args): pass
    def set_index_high(self, *_args): pass


class SensorManager:
    """
    Open and start the VL53L1X ranging sensor
    """

    def __init__(self, address):
        self.address = address
        
        # This will raise OSError if there's no /dev/i2c-1.
        self.tof = TofSensor(bus=1, address=address)
        self.log = logging.getLogger(f'tof.{address:x}')
        self.enabled = True


    def configure(self, timing, inter, roi=None):
        # this will fail if there's nothing on the bus
        self.tof.init(True)  # True means set for 2.8V

        # self.tof.stop_streaming()

        self.log.debug('set mode')
        self.tof.set_long_distance_mode(False)  # short distance mode for faster readings

        # Lower timing budgets allow for faster updates, but sacrifice accuracy
        self.log.debug('set timing=%s ms', timing)
        self.tof.set_timing_budget_ms(timing)

        # inter-measurement period must be higher than timing budget so use max()
        self.log.debug('set inter=%s ms', inter)
        self.tof.set_inter_measurement_period_ms(max(timing, inter))

        roi_center = self.tof.get_roi_center()
        self.log.debug('roi center is %s', roi_center)

        roi_size = self.tof.get_roi()
        self.log.debug('roi size is %s', roi_size)

        if roi is not None:
            self.log.debug('set roi size=%s', roi)
            self.tof.set_roi(*roi)

        self.log.debug('start streaming')
        self.tof.start_streaming()


    def is_running(self):
        return self.tof.is_running()


    def read(self, timing, inter, roi=None, callback=None):
        '''Run loop reading from the sensor, via a blocking call to
        get_reading(), which retrieves readings from the sensor thread.'''
        try:
            self.log.info('reading: timing=%s inter=%s', timing, inter)
            ts_last = time.monotonic() # try to match the one Rust uses...
            self.enabled = True
            running = False
            warned = None # flag when we've reported a problem, to avoid spamming
            while self.enabled:
                if not running:
                    # Try reconfiguring sensor.  If it's present it will
                    # end up configured properly and streaming.  If it's
                    # not present or there's a problem, we'll get an
                    # exception, then try again after a brief pause.
                    try:
                        self.configure(timing, inter, roi)
                    except Exception as ex:
                        if not warned or warned is not ex.__class__:
                            warned = ex.__class__
                            warned = None
                            self.log.error("Error: %s", ex)

                        time.sleep(0.2)
                        continue
                    running = True
                    warned = False

                    self.log.info("ACTIVATED")

                try:
                    # This should block until there's a reading or error.
                    reading = self.tof.get_reading()

                    if reading is None:
                        running = False
                        self.log.warning("DEACTIVATED")
                        continue

                except Exception as ex:
                    if not warned:
                        warned = True
                        self.log.error("Error: %s", ex)

                    running = False
                    continue

                else:
                    ts, distance, status = reading
                    # self.log.debug('reading: %s,%s,%s', ts, distance, status)

                    delta = ts - ts_last
                    ts_last = ts

                    if callback is not None:
                        callback(ts, distance, status, delta)

                    time.sleep(0.001)  # Small sleep to prevent CPU spinning
                    # (although the new Rust code should block us anyway
                    # so in theory this is no longer required)

        finally:
            self.tof.stop_streaming()


    def shutdown(self):
        self.enabled = False
        self.tof.stop_streaming()


# try to import the real thing when on a Pi
if re.search(r'(?m)^Model\s+:.*Raspberry', open('/proc/cpuinfo').read()):
    from vl53l1x import TofSensor, ThreadError, SetPins
else:
    logging.warning('simulating TOF sensor')
