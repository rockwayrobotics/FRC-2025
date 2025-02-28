import math
import numpy as np
import scipy as sp

# distances in mm
min_distance = 150
max_distance = 750

# number of elements in full buffer
buffer_size = 1000

class CornerDetector:
    def __init__(
        self, slope_threshold, large_window=25, small_window=10, flat_slope_threshold=60
    ):
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

    def reset(self):
        self.start_index = 0
        self.end_index = 0
        self.corner_timestamp = None
        self.corner_angle = None

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

    def add_record(self, timestamp, distance, speed=None):
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
        """
        print(
            "First regression",
            self.data[self.start_index : self.start_index + self.small_window, 1],
            first_regression.slope,
            first_regression.rvalue**2,
            first_regression.pvalue,
            first_regression.stderr,
        )
        """
        second_regression = sp.stats.linregress(
            self.data[self.end_index - self.small_window : self.end_index, 0],
            self.data[self.end_index - self.small_window : self.end_index, 1],
        )
        """
        print(
            "Second regression",
            self.data[self.end_index - self.small_window : self.end_index, 1],
            second_regression.slope,
            second_regression.rvalue**2,
            second_regression.pvalue,
            second_regression.stderr,
        )
        """
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
            self.corner_timestamp = self.intersect_walls(
                first_regression, second_regression
            )
            if speed is not None:
                self.corner_angle = math.atan2(second_regression.slope, speed * 1000)
            """
            print(
                f" *** Found corner: {self.corner_timestamp} {first_regression.slope} {second_regression.slope} ***"
            )
            """
            return

    def intersect_walls(self, first, second):
        if abs(first.slope - second.slope) < 1e-6:
            # Should be impossible, walls are parallel
            return

        return (second.intercept - first.intercept) / (first.slope - second.slope)
