package frc.robot.simulation;

import edu.wpi.first.util.CircularBuffer;
import frc.robot.simulation.PiSimulation.TimeDistance;

public class LinearRegression {
  public double intercept;
  public double slope;
  public double r2value;
  public double yRange;
  public double rmse; // root-mean-square-error

  /**
   * Linear regression adapted from
   * https://algs4.cs.princeton.edu/code/edu/princeton/cs/algs4/LinearRegression.java.html
   *
   * @param buffer     Circular buffer of time/distance
   * @param startIndex inclusive
   * @param endIndex   not inclusive
   */
  public LinearRegression(CircularBuffer<TimeDistance> buffer, int startIndex, int endIndex) {
    int n = endIndex - startIndex;
    double yMin = Double.POSITIVE_INFINITY;
    double yMax = Double.NEGATIVE_INFINITY;
    double xSum = 0.0, ySum = 0.0;
    for (int i = startIndex; i < endIndex; i++) {
      var value = buffer.get(i);
      xSum += value.time();
      ySum += value.distance();
      yMin = Math.min(yMin, value.distance());
      yMax = Math.max(yMax, value.distance());
    }
    double xMean = xSum / n;
    double yMean = ySum / n;
    yRange = yMax - yMin;

    double x2Differences = 0.0, y2Differences = 0.0, xyDifferences = 0.0;
    for (int i = startIndex; i < endIndex; i++) {
      var value = buffer.get(i);
      x2Differences += Math.pow(value.time() - xMean, 2);
      y2Differences += Math.pow(value.distance() - yMean, 2);
      xyDifferences += (value.time() - xMean) * (value.distance() - yMean);
    }
    slope = xyDifferences / x2Differences;
    intercept = yMean - slope * xMean;

    double ssRegression = 0.0; // regression sum of squares
    for (int i = startIndex; i < endIndex; i++) {
      var value = buffer.get(i);
      double fit = slope * value.time() + intercept;
      ssRegression += Math.pow(fit - yMean, 2);
    }
    r2value = y2Differences == 0 ? 0 : ssRegression / y2Differences;
    rmse = Math.sqrt(ssRegression / n);
  }
}
