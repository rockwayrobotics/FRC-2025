package frc.robot.simulation;

import edu.wpi.first.util.CircularBuffer;
import frc.robot.simulation.PiSimulation.TimeDistance;

public class LinearRegression {
  public final double intercept;
  public final double slope;
  public final double r2value;

  /**
   * Linear regression
   * @param buffer Circular buffer of time/distance
   * @param startIndex inclusive
   * @param endIndex not inclusive
   */
  public LinearRegression(CircularBuffer<TimeDistance> buffer, int startIndex, int endIndex) {
    // Adapted from https://algs4.cs.princeton.edu/code/edu/princeton/cs/algs4/LinearRegression.java.html
    int n = endIndex - startIndex;

    double xSum = 0.0, ySum = 0.0, x2Sum = 0.0;
    for (int i = startIndex; i < endIndex; i++) {
      var value = buffer.get(i);
      xSum += value.time();
      x2Sum += Math.pow(value.time(), 2);
      ySum = value.distance();
    }
    double xMean = xSum / n;
    double yMean = ySum / n;

    double x2Differences = 0.0, y2Differences = 0.0, xyDifferences = 0.0;
    for (int i = startIndex; i < endIndex; i++) {
      var value = buffer.get(i);
      x2Differences += Math.pow(value.time() - xMean,2 );
      y2Differences += Math.pow(value.distance() - yMean,2 );
      xyDifferences += (value.time() - xMean) * (value.distance() - yMean);
    }
    slope = xyDifferences / x2Differences;
    intercept = yMean - slope * xMean;

    double ssRegression = 0.0;
    for (int i = startIndex; i <endIndex; i++) {
      var value = buffer.get(i);
      double fit = slope * value.time() + intercept;
      ssRegression += Math.pow(fit - yMean, 2);
    }
    r2value = ssRegression / x2Differences;
  }
}
