import logging
import math
import numpy as np
import scipy as sp
import time

# distances in mm
min_distance = 50
max_distance = 1000

# number of elements in full buffer
buffer_size = 1000

class CornerDetector:
    def __init__(
        self, slope_threshold, large_window=25, small_window=10, flat_slope_threshold=60,
    ):
        self.log = logging.getLogger('cd')
        # Number of samples in window
        self.large_window = large_window
        # Number of samples at start and end of window to look for linearity
        self.small_window = small_window
        # Slopes less than this are considered "linear" no matter what rvalue they have
        # This is sort of a hack because rvalues are not reliable for noisy data with an almost-flat slope
        self.flat_slope_threshold = flat_slope_threshold
        # Difference between slopes should be at least this
        self.slope_threshold = slope_threshold
        self.data = np.zeros((buffer_size, 2), dtype="f")

        self.reset()

        self.log.debug('initialized')

    def reset(self):
        # self.log.debug('reset')
        self.start_index = 0
        self.end_index = 0
        self.corner_timestamp = None
        self.corner_angle = None
        self.corner_dist = 0
        self.timing = np.zeros(50)
        self.timing_index = 0
        self.last_slope_difference = None
        self.stored_corner_timestamp = None
        self.stored_corner_angle = None
        self.stored_corner_dist = 0

    def found_corner(self):
        return self.corner_timestamp is not None

    def shift_buffer(self):
        # According to https://stackoverflow.com/questions/30399534/shift-elements-in-a-numpy-array
        # reallocating is likely faster
        result = np.zeros((buffer_size, 2), dtype="f")
        count = self.end_index - self.start_index
        result[: count] = self.data[self.start_index : self.end_index]
        self.start_index = 0
        self.end_index = count #self.large_window
        self.data = result

    def _add_record(self, timestamp, distance, speed=None):
        if distance > max_distance:
            # Reset only if distance is greater than max
            self.reset()
            return

        if distance < min_distance:
            # Ignore data closer than minimum, but do not reset
            return

        if self.end_index >= buffer_size - 1:
            self.shift_buffer()

        i = self.end_index
        self.data[i][0] = timestamp
        self.data[i][1] = distance
        self.end_index += 1

        if self.end_index - self.start_index < self.large_window:
            return

        # This is sort of wasteful, because we calculate the same linear regression
        # multiple times
        first_regression = sp.stats.linregress(
            self.data[self.start_index : self.start_index + self.small_window, 0],
            self.data[self.start_index : self.start_index + self.small_window, 1],
        )

        self.log.debug("first regression: %s, %s, %s, %s, %s",
            self.data[self.start_index : self.start_index + self.small_window, 1],
            first_regression.slope,
            first_regression.rvalue**2,
            first_regression.pvalue,
            first_regression.stderr,
        )

        second_regression = sp.stats.linregress(
            self.data[self.end_index - self.small_window : self.end_index, 0],
            self.data[self.end_index - self.small_window : self.end_index, 1],
        )

        self.log.debug("second regression: %s, %s, %s, %s, %s",
            self.data[self.end_index - self.small_window : self.end_index, 1],
            second_regression.slope,
            second_regression.rvalue**2,
            second_regression.pvalue,
            second_regression.stderr,
        )
        self.start_index += 1

        if (
            # rvalues are good for linearity at "large" slopes, but are useless for slopes near 0
            (
                first_regression.rvalue**2 > 0.8
                or abs(first_regression.slope) < self.flat_slope_threshold
            )
            and (
                second_regression.rvalue**2 > 0.8
                or abs(second_regression.slope) < self.flat_slope_threshold
            )
            # The way we are reading the values, we always expect the slope to increase
            and second_regression.slope - first_regression.slope > self.slope_threshold
        ):
            slope_difference = second_regression.slope - first_regression.slope
            if self.last_slope_difference is not None and slope_difference <= self.last_slope_difference:
                # The best corner is found when the slope difference is maximized
                self.corner_timestamp = self.stored_corner_timestamp
                self.corner_dist = self.stored_corner_dist
                if self.stored_corner_angle is not None:
                    self.corner_angle = self.stored_corner_angle

                self.log.info("CORNER: %.3f,%.1f,%.1f",
                    self.corner_timestamp, first_regression.slope, second_regression.slope)

                return

            self.last_slope_difference = slope_difference
            (self.stored_corner_timestamp, self.stored_corner_dist) = self.intersect_walls(
                first_regression, second_regression
            )
            if speed is not None:
                self.stored_corner_angle = math.atan2(second_regression.slope, speed * 1000)

    def intersect_walls(self, first, second):
        if abs(first.slope - second.slope) < 1e-6:
            # Should be impossible, walls are parallel
            return

        corner_ts = (second.intercept - first.intercept) / (first.slope - second.slope)
        corner_dist = first.slope * corner_ts + first.intercept
        return (corner_ts, corner_dist)

    def add_record(self, timestamp, distance, speed=None):
        start_ts = time.monotonic()

        try:
            return self._add_record(timestamp, distance, speed)
        finally:
            elapsed = time.monotonic() - start_ts
            self.timing[self.timing_index] = elapsed
            self.timing_index = (self.timing_index + 1) % len(self.timing)

    def log_timing(self):
        self.log.info("timing: min=%.1f max=%.1f mean=%.1f std=%.1f",
            self.timing.min() * 1000,
            self.timing.max() * 1000,
            self.timing.mean() * 1000,
            self.timing.std() * 1000,
        )

