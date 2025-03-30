import random
import re
import time

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

# try to import the real thing when on a Pi
if re.search(r'(?m)^Model\s+:.*Raspberry', open('/proc/cpuinfo').read()):
    from vl53l1x import TofSensor, ThreadError, SetPins
