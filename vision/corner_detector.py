import numpy as np
import scipy as sp

# distances in mm
min_distance = 150
max_distance = 600

class Phase:
  SEEKING_FIRST_WALL = 0
  SEEKING_TRANSITION = 1
  SEEKING_SECOND_WALL = 2
  FOUND_CORNER = 3

class CornerDetector:
  def __init__(self, slope_threshold, window_size = 3, slope_window_size = 3):
    self.slope_threshold = slope_threshold
    self.window_size = window_size
    self.slope_window_size = slope_window_size
    # Create a 2d array of length 1000 that has space for timestamp, distance_in_mm
    self.data = np.zeros((1000, 2), dtype='f')
    self.slopes = np.zeros((self.slope_window_size, 1), dtype='f')
    self.reset()

  def reset(self):
    self.first_slope = None
    # This is the actual amount of valid data stored in the array
    self.index = 0
    self.slope_count = 0
    self.first_wall_slope = 0.
    self.corner_timestamp = 0.
    self.phase = Phase.SEEKING_FIRST_WALL

  def found_corner(self):
    return self.phase == Phase.FOUND_CORNER

  def add_record(self, timestamp, distance):
    if self.phase == Phase.FOUND_CORNER:
      return

    if distance < min_distance or distance > max_distance:
      # Throw away everything if we receive value out of range
      self.reset()
      return

    i = self.index
    self.data[i][0] = timestamp
    self.data[i][1] = distance
    self.index = self.index + 1
    
    if i + 1 < self.window_size:
      return

    regression = sp.stats.linregress(self.data[self.index-self.window_size:self.index,0], self.data[self.index-self.window_size:self.index,1])

    if self.phase == Phase.SEEKING_FIRST_WALL:
      if self.slope_count >= self.slope_window_size:
        average_slope = np.average(self.slopes)
        if abs(regression.slope - average_slope) < self.slope_threshold:
          self.phase = Phase.SEEKING_TRANSITION
          wall_regression = sp.stats.linregress(self.data[self.index - (self.slope_window_size + self.window_size - 1):self.index,0], self.data[self.index - (self.slope_window_size + self.window_size - 1):self.index,1])
          self.first_wall_slope = wall_regression.slope
          self.first_wall_intercept = wall_regression.intercept
          self.slope_count = 0
          return
      self.slopes[i % self.slope_window_size] = regression.slope
      self.slope_count = self.slope_count + 1
    elif self.phase == Phase.SEEKING_TRANSITION:
      if abs(regression.slope - self.first_wall_slope) > self.slope_threshold:
        self.phase = Phase.SEEKING_SECOND_WALL
        self.transition_timestamp = timestamp
        return
    elif self.phase == Phase.SEEKING_SECOND_WALL:
      if self.slope_count >= self.slope_window_size:
        average_slope = np.average(self.slopes)
        if abs(regression.slope - average_slope) < self.slope_threshold:
          self.phase = Phase.FOUND_CORNER
          # Could slightly correct it with the regression.slope
          wall_regression = sp.stats.linregress(self.data[self.index - (self.slope_window_size + self.window_size - 1):self.index,0], self.data[self.index - (self.slope_window_size + self.window_size - 1):self.index,1])
          self.second_wall_slope = wall_regression.slope
          self.second_wall_intercept = wall_regression.intercept
          self.intersect_walls()
          self.slope_count = 0
          return
      self.slopes[i % self.slope_window_size] = regression.slope
      self.slope_count = self.slope_count + 1
     
  def intersect_walls(self):
    if abs(self.first_wall_slope - self.second_wall_slope) < 1e-6:
      # Should be impossible, walls are parallel
      return

    self.corner_timestamp = (self.second_wall_intercept - self.first_wall_intercept) / (self.first_wall_slope - self.second_wall_slope)
