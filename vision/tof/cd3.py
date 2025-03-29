import math

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
        self.speed = 450 # default...

        # This eliminates a weird ~400ms pause on our first corner, which may be from lazy
        # loading a library related to this routine. Calling it here appears to
        # move the load up to this point so it won't affect us later.
        _ = sp.stats.siegelslopes([0], [0])

        super().__init__()

        self.log.info('params: st=%.1f, sw=%s, ew=%s, fst=%s',
            self.slope_threshold, self.start_window, self.end_window, self.flat_slope_threshold)

    def reset(self):
        super().reset()

        self.start_index = 0
        self.end_index = 0
        self.corner_timestamp = None
        self.corner_angle = None
        self.corner_dist = 0
        # represents a +/- noise value we expect in typical readings
        self.signal_noise = 10.0
        # represents how many standard deviations we allow 
        self.sigma_threshold = 1.9
        self.recalc_slopes(self.speed)

    def recalc_slopes(self, speed):
        changed = speed != self.speed
        self.speed = speed
        # what sort of slope do we expected
        self.target_slope = math.tan(60 * math.pi / 180) * self.speed
        # this determines a range around the target slope by multiplying and dividing by it
        # empirical: 20250328-163325 at 1810.549 we had a slope delta of 991
        # with speed 450 and tolerance 0.8 we rejected this (range 623..974)
        # so changed to 0.75 (range 584..1039)
        self.slope_tolerance = 0.75
        self.slope_diff_min = self.target_slope * self.slope_tolerance
        self.slope_diff_max = self.target_slope * (1 / self.slope_tolerance)
        # if changed:
        #     self.log.debug('speed now %s: recalc slopes: %.0f,%.0f,%.0f',
        #         self.speed,
        #         self.target_slope, self.slope_diff_min, self.slope_diff_max)

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
        if self.debug:
            self.log.debug('--------------')
            self.log.debug("add_record %s,%s: %s..%s", timestamp, distance,
                self.start_index, self.end_index)

        # handle change in speed
        if abs(speed - self.speed) >= 1.0:
            self.recalc_slopes(speed)
            
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
            x0 = self.data[self.start_index : self.start_index + self.start_window, 0]
            y0 = self.data[self.start_index : self.start_index + self.start_window, 1]
            x1 = self.data[self.end_index - self.end_window : self.end_index, 0]
            y1 = self.data[self.end_index - self.end_window : self.end_index, 1]
            if self.debug:
                self.log.debug('x0 %s, y0 %s', x0, y0)
                self.log.debug('x1 %s, y1 %s', x1, y1)

            # if x0[-1] > 1811.68 and x0[-1] < 1811.8:
            #     breakpoint()

            d0 = np.diff(y0)
            d1 = np.diff(y1)

            s0 = d0.mean() * len(d0) / (x0[-1] - x0[0])
            if s0 >= 0:
                if self.debug:
                    self.log.debug('first wall has positive slope!')
                return
            
            s1 = d1.mean() * len(d1) / (x1[-1] - x1[0])
            sdiff = s1 - s0

            if self.debug:
                self.log.debug("%.3f: dist=%.3f", timestamp, distance)
            
            if sdiff < self.slope_diff_min or sdiff > self.slope_diff_max:
                return

            # count how many outliers points we have
            q0 = np.count_nonzero(np.abs(d0 - d0.mean()) > d0.std() * self.sigma_threshold)
            q1 = np.count_nonzero(np.abs(d1 - d1.mean()) > d1.std() * self.sigma_threshold)

            # this shouldn't reject the whole thing, just maybe tag individual points as outliers
            # but for the moment it helps by helping for "rounded corner" points into the window gap
            if d1.max() - d1.mean() > self.signal_noise or d1.mean() - d1.min() > self.signal_noise:
                return
            span1 = d1.max() - d1.mean()

            if q0 < 3 and q1 < 3:
                new_first = sp.stats.siegelslopes(y0, x0)
                if new_first.slope >= 0:
                    return
            
                new_second = sp.stats.siegelslopes(y1, x1)
                (self.corner_timestamp, self.corner_dist) = self.intersect_walls(
                    new_first, new_second
                )
                if speed is not None:
                    self.corner_angle = math.atan2(new_second.slope, speed)

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


