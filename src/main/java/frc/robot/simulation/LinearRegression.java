package frc.robot.simulation;

import edu.wpi.first.util.CircularBuffer;
import frc.robot.simulation.PiSimulation.TimeDistance;

public class LinearRegression {
  public double intercept;
  public double slope;
  public double r2value;

  /**
   * Linear regression
   * @param buffer Circular buffer of time/distance
   * @param startIndex inclusive
   * @param endIndex not inclusive
   */
  public LinearRegression(CircularBuffer<TimeDistance> buffer, int startIndex, int endIndex) {
    int n = endIndex - startIndex;
    double[] x = new double[n];
    double[] y = new double[n];
    int a = 0;
    for (int i = startIndex; i < endIndex; i++) {
      var value = buffer.get(i);
      x[a] = value.time();
      y[a] = value.distance();
      a++;
    }
    doRegression(x, y);

    /*
    // Adapted from https://algs4.cs.princeton.edu/code/edu/princeton/cs/algs4/LinearRegression.java.html

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
    */
  }

  public void doRegression(double[] x, double[] y) {
      if (x.length != y.length) {
          throw new IllegalArgumentException("array lengths are not equal");
      }
      int n = x.length;

      // first pass
      double sumx = 0.0, sumy = 0.0, sumx2 = 0.0;
      for (int i = 0; i < n; i++) {
          sumx  += x[i];
          sumx2 += x[i]*x[i];
          sumy  += y[i];
      }
      double xbar = sumx / n;
      double ybar = sumy / n;

      // second pass: compute summary statistics
      double xxbar = 0.0, yybar = 0.0, xybar = 0.0;
      for (int i = 0; i < n; i++) {
          xxbar += (x[i] - xbar) * (x[i] - xbar);
          yybar += (y[i] - ybar) * (y[i] - ybar);
          xybar += (x[i] - xbar) * (y[i] - ybar);
      }
      slope  = xybar / xxbar;
      intercept = ybar - slope * xbar;

      // more statistical analysis
      double rss = 0.0;      // residual sum of squares
      double ssr = 0.0;      // regression sum of squares
      for (int i = 0; i < n; i++) {
          double fit = slope*x[i] + intercept;
          rss += (fit - y[i]) * (fit - y[i]);
          ssr += (fit - ybar) * (fit - ybar);
      }

      int degreesOfFreedom = n-2;
      r2value    = ssr / yybar;
      double svar  = rss / degreesOfFreedom;
  }
}
