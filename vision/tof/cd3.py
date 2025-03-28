import logging
import math
import time

import numpy as np
import scipy as sp

from .corner_detector import BaseCornerDetector

class CornerDetector(BaseCornerDetector):
    name = 'v3'
    
    def __init__(
        self, slope_threshold, large_window=25, small_window=10, flat_slope_threshold=60,
    ):
        # Number of samples in window
        self.large_window = large_window

        # Number of samples at start and end of window to look for linearity
        self.start_window = small_window
        self.end_window = small_window

        # Slopes less than this are considered "linear" no matter what rvalue they have
        # This is sort of a hack because rvalues are not reliable for noisy data with an almost-flat slope
        self.flat_slope_threshold = flat_slope_threshold

        # Difference between slopes should be at least this
        self.slope_threshold = slope_threshold

        self.data = np.zeros((self.BUFFER_SIZE, 2), dtype="f")

        super().__init__()

    def reset(self):
        super().reset()

        self.start_index = 0
        self.end_index = 0
        self.corner_timestamp = None
        self.corner_angle = None
        self.corner_dist = 0

    def shift_buffer(self):
        # According to https://stackoverflow.com/questions/30399534/shift-elements-in-a-numpy-array
        # reallocating is likely faster
        result = np.zeros((self.BUFFER_SIZE, 2), dtype="f")
        count = self.end_index - self.start_index
        result[: count] = self.data[self.start_index : self.end_index]
        self.start_index = 0
        self.end_index = count #self.large_window
        self.data = result

    def _add_record(self, timestamp, distance, speed=None):
        if distance > self.MAX_DISTANCE:
            # Reset only if distance is greater than max
            self.reset()
            return

        if distance < self.MIN_DISTANCE:
            # Ignore data closer than minimum, but do not reset
            return

        if self.end_index >= len(self.data) - 1:
            self.shift_buffer()

        i = self.end_index
        self.data[i][0] = timestamp
        self.data[i][1] = distance
        self.end_index += 1

        if self.end_index - self.start_index < self.large_window:
            return

        try:
            # # This is sort of wasteful, because we calculate the same linear regression
            # # multiple times
            # first_regression = sp.stats.linregress(
            #     self.data[self.start_index : self.start_index + self.small_window, 0],
            #     self.data[self.start_index : self.start_index + self.small_window, 1],
            # )

            # # breakpoint()
            # self.log.debug("%s: first regression: %s, %s, %s, %s, %s",
            #     timestamp,
            #     self.data[self.start_index : self.start_index + self.small_window, 1],
            #     first_regression.slope,
            #     first_regression.rvalue**2,
            #     first_regression.pvalue,
            #     first_regression.stderr,
            # )

            # second_regression = sp.stats.linregress(
            #     self.data[self.end_index - self.small_window : self.end_index, 0],
            #     self.data[self.end_index - self.small_window : self.end_index, 1],
            # )

            # self.log.debug("%s: second regression: %s, %s, %s, %s, %s",
            #     timestamp,
            #     self.data[self.end_index - self.small_window : self.end_index, 1],
            #     second_regression.slope,
            #     second_regression.rvalue**2,
            #     second_regression.pvalue,
            #     second_regression.stderr,
            # )
            # self.start_index += 1

            # if (
            #     # rvalues are good for linearity at "large" slopes, but are useless for slopes near 0
            #     (
            #         first_regression.rvalue**2 > 0.95
            #         or abs(first_regression.slope) < self.flat_slope_threshold
            #     )
            #     and (
            #         second_regression.rvalue**2 > 0.95
            #         or abs(second_regression.slope) < self.flat_slope_threshold
            #     )
            #     # The way we are reading the values, we always expect the slope to increase
            #     and second_regression.slope - first_regression.slope > self.slope_threshold
            # ):

        
            x0 = self.data[self.start_index : self.start_index + self.start_window, 0]
            y0 = self.data[self.start_index : self.start_index + self.start_window, 1]
            x1 = self.data[self.end_index - self.end_window : self.end_index, 0]
            y1 = self.data[self.end_index - self.end_window : self.end_index, 1]

            d0 = np.diff(y0)
            d1 = np.diff(y1)
            # if len(d0) == 0 or len(d1) == 0:
            #     breakpoint()

            s0 = d0.mean() * len(d0) / (x0[-1] - x0[0])
            if s0 >= 0:
                return
            
            s1 = d1.mean() * len(d1) / (x1[-1] - x1[0])
            sdiff = s1 - s0
            NOMSLOPE = 780 # based on 450mm/s
            if sdiff < NOMSLOPE * .80 or sdiff > NOMSLOPE / 0.80:
                return

            TOLERANCE = 1.9
            q0 = np.count_nonzero(np.abs(d0 - d0.mean()) > d0.std() * TOLERANCE)
            q1 = np.count_nonzero(np.abs(d1 - d1.mean()) > d1.std() * TOLERANCE)
            NOISE_THRESHOLD = 7.0
            if d1.max() - d1.mean() > NOISE_THRESHOLD or d1.mean() - d1.min() > NOISE_THRESHOLD:
                return
            span1 = d1.max() - d1.mean()

            if q0 < 3 and q1 < 3:
                # breakpoint()
                
                new_first = sp.stats.siegelslopes(y0, x0)
                if new_first.slope >= 0:
                    return
            
                new_second = sp.stats.siegelslopes(y1, x1)
                (self.corner_timestamp, self.corner_dist) = self.intersect_walls(
                    new_first, new_second
                )
                if speed is not None:
                    self.corner_angle = math.atan2(new_second.slope, speed * 1000)

                self.log.info("CORNER: %.3f,%.1f,%.1f gap %s..%s slope-change %.0f q0=%s q1=%s span1=%.1f",
                    self.corner_timestamp, new_first.slope, new_second.slope,
                    x0[-1], x1[0], new_second.slope - new_first.slope,
                    q0, q1, span1)
                return
        finally:
            self.start_index += 1

    def intersect_walls(self, first, second):
        if abs(first.slope - second.slope) < 1e-6:
            # Should be impossible, walls are parallel
            return

        corner_ts = (second.intercept - first.intercept) / (first.slope - second.slope)
        corner_dist = first.slope * corner_ts + first.intercept
        return (corner_ts, corner_dist)


