package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.util.CircularBuffer;

public class CornerDetector {
  private static class LineSegment {
    final double slope;
    final double intercept;
    final double startTime;
    final double endTime;

    LineSegment(double slope, double intercept, double startTime, double endTime) {
      this.slope = slope;
      this.intercept = intercept;
      this.startTime = startTime;
      this.endTime = endTime;
    }

    void print() {
      System.out.printf("Wall %.2f %.2f %.2f %.2f%n", slope, intercept, startTime, endTime);
    }

    Optional<Double> intersect(LineSegment other) {
      if (Math.abs(slope - other.slope) < 1e-6) {
        return Optional.empty(); // lines are parallel
      }

      // Calculate intersection point
      double x = (other.intercept - intercept) / (slope - other.slope);

      // Check if intersection is within the time range of both line segments
      if ((x >= startTime && x <= other.endTime) ||
          (x >= other.startTime && x <= endTime)) {
        return Optional.of(x);
      }

      return Optional.empty();
    }
  }

  private final CircularBuffer<Double> rawDistances;
  private final CircularBuffer<Double> timestamps;
  private final CircularBuffer<Double> smoothedDistances;
  private final CircularBuffer<Double> slopes;
  private final int windowSize;
  private final double slopeThreshold;
  private final double slopeStabilityThreshold;
  private final int requiredStablePoints;
  private double lastCornerTimestamp = 0;
  private double wallSlope = 0;

  // Track the detection phases
  private enum DetectionPhase {
    SEEKING_FIRST_STABLE_SLOPE,
    SEEKING_TRANSITION,
    SEEKING_SECOND_STABLE_SLOPE,
    FOUND_CORNER
  }

  private DetectionPhase currentPhase = DetectionPhase.SEEKING_FIRST_STABLE_SLOPE;
  private double firstStableSlope = 0;
  private LineSegment firstWall = null;

  public CornerDetector(int windowSize, double slopeThreshold,
      double slopeStabilityThreshold,
      int requiredStablePoints) {
    this.windowSize = windowSize;
    this.slopeThreshold = slopeThreshold;
    this.slopeStabilityThreshold = slopeStabilityThreshold;
    this.requiredStablePoints = requiredStablePoints;

    this.rawDistances = new CircularBuffer<>(windowSize);
    this.smoothedDistances = new CircularBuffer<>(windowSize);
    this.timestamps = new CircularBuffer<>(windowSize);
    this.slopes = new CircularBuffer<>(windowSize - 1);
  }

  public void recordDistance(double distance, double timestamp) {
    // Add new measurement to buffers
    rawDistances.addFirst(distance);
    timestamps.addFirst(timestamp);
    System.out.printf("%.2f, %.2f%n", distance, timestamp);

    if (!isBufferFull()) {
      return;
    }

    // Apply smoothing and calculate new slope
    double smoothedDistance = smoothMeasurement();
    smoothedDistances.addFirst(smoothedDistance);

    if (smoothedDistances.size() < 2) {
      return;
    }

    double currentSlope = calculateSlope(0);
    if (!Double.isFinite(currentSlope)) {
      return;
    }
    slopes.addFirst(currentSlope);
    System.out.printf("Slope: %.2f%n", currentSlope);

    if (slopes.size() < requiredStablePoints) {
      return;
    }

    // Process based on current detection phase
    switch (currentPhase) {
      case SEEKING_FIRST_STABLE_SLOPE:
        if (hasStableSlope()) {
          firstStableSlope = calculateMeanSlope();
          firstWall = fitLine();
          firstWall.print();
          currentPhase = DetectionPhase.SEEKING_TRANSITION;
        }
        break;

      case SEEKING_TRANSITION:
        // Check if current slope is significantly different from first stable slope
        System.out.printf("Slope: %.2f change from %.2f%n", currentSlope, firstStableSlope);
        if (Math.abs(currentSlope - firstStableSlope) > slopeThreshold) {
          System.out.printf("Transition time chosen: %.2f%n", timestamp);
          currentPhase = DetectionPhase.SEEKING_SECOND_STABLE_SLOPE;
        }
        break;

      case SEEKING_SECOND_STABLE_SLOPE:
        if (hasStableSlope()) {
          double secondStableSlope = calculateMeanSlope();

          System.out.printf("Checking second stable slope: %.2f%n", secondStableSlope);
          // Verify the two stable slopes are different enough
          if (Math.abs(secondStableSlope - firstStableSlope) > slopeThreshold) {
            LineSegment secondWall = fitLine();
            secondWall.print();
            Optional<Double> cornerTime = firstWall.intersect(secondWall);
            if (cornerTime.isPresent()) {
              lastCornerTimestamp = cornerTime.get();
              System.out.printf("Found corner: %.2f%n", lastCornerTimestamp);
              wallSlope = secondStableSlope;
              System.out.printf("Using wall slope: %.2f instead of %.2f%n", wallSlope, secondStableSlope);
              currentPhase = DetectionPhase.FOUND_CORNER;
              firstWall = null;
            }
          }
        }
        break;

      case FOUND_CORNER:
        // Do nothing, we already found a corner.
        break;
    }

    return;
  }

  public Optional<Double> getCornerTimestamp() {
    if (currentPhase != DetectionPhase.FOUND_CORNER) {
      return Optional.empty();
    }
    return Optional.of(lastCornerTimestamp);
  }

  public double getWallAngle(double speedMetersPerSecond) {
    System.out.printf("WallAngle: %.3f, %.3f%n", wallSlope, 1000.0 * speedMetersPerSecond);
    // Wall slope is in mm, but we want meters
    return Math.atan2(wallSlope, 1000.0 * speedMetersPerSecond);
  }

  private double smoothMeasurement() {
    double sum = 0;
    int count = Math.min(windowSize, rawDistances.size());
    for (int i = 0; i < count; i++) {
      sum += rawDistances.get(i);
    }
    return sum / count;
  }

  private LineSegment fitLine() {
    // Simple linear regression
    double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    double startTime = timestamps.get(requiredStablePoints - 1);
    double endTime = timestamps.get(0);

    for (int i = 0; i < requiredStablePoints; i++) {
      double x = timestamps.get(i);
      double y = rawDistances.get(i);
      sumX += x;
      sumY += y;
      sumXY += x * y;
      sumXX += x * x;
    }

    double n = requiredStablePoints;
    double slope = (n * sumXY - sumX * sumY) / (n * sumXX - sumX * sumX);
    double intercept = (sumY - slope * sumX) / n;

    return new LineSegment(slope, intercept, startTime, endTime);
  }

  private boolean hasStableSlope() {
    double meanSlope = calculateMeanSlope();

    // Check if all recent slopes are within threshold of mean
    for (int i = 0; i < requiredStablePoints; i++) {
      if (Math.abs(slopes.get(i) - meanSlope) > slopeStabilityThreshold) {
        return false;
      }
    }

    return true;
  }

  private double calculateMeanSlope() {
    double sum = 0;
    for (int i = 0; i < requiredStablePoints; i++) {
      sum += slopes.get(i);
    }
    return sum / requiredStablePoints;
  }

  private double calculateSlope(int startIndex) {
    double dx = timestamps.get(startIndex) - timestamps.get(startIndex + 1);
    double dy = smoothedDistances.get(startIndex) - smoothedDistances.get(startIndex + 1);
    return dy / dx;
  }

  private boolean isBufferFull() {
    return rawDistances.size() >= windowSize &&
        timestamps.size() >= windowSize;
  }

  /**
   * Get the current detection phase (useful for debugging)
   */
  public DetectionPhase getCurrentPhase() {
    return currentPhase;
  }

  /**
   * Reset the detector state (useful if you need to restart detection)
   */
  public void reset() {
    // FIXME: Should we empty buffers?
    currentPhase = DetectionPhase.SEEKING_FIRST_STABLE_SLOPE;
    firstStableSlope = 0;
  }
}