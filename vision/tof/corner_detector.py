import logging
import time

import numpy as np

class BaseCornerDetector:
    # distances in mm
    MIN_DISTANCE = 50
    MAX_DISTANCE = 1000

    # number of elements in full buffer
    BUFFER_SIZE = 1000

    name = 'base' # change in subclasses

    @staticmethod
    def timestamp():
        return time.monotonic()

    def __str___(self):
        return f'CD{self.name}'

    def __init__(self):
        self.log = logging.getLogger('cd')
        self.debug = False
        self.reset()
        # self.log.debug('initialized')

    def reset(self):
        self.timing = np.zeros(50)
        self.timing_index = 0
        self.corner_timestamp = None

    def add_record(self, timestamp, distance, speed=None):
        start_ts = self.timestamp()

        try:
            return self._add_record(timestamp, distance, speed)
        finally:
            elapsed = self.timestamp() - start_ts
            self.timing[self.timing_index] = elapsed
            self.timing_index = (self.timing_index + 1) % len(self.timing)
            if elapsed > 0.01:
                self.log.warning('long calc! %.3fs', elapsed)

    def found_corner(self):
        return self.corner_timestamp is not None

    def log_timing(self):
        self.log.info("timing: min=%.1f max=%.1f mean=%.1f std=%.1f",
            self.timing.min() * 1000,
            self.timing.max() * 1000,
            self.timing.mean() * 1000,
            self.timing.std() * 1000,
        )

