package frc.robot.simulation;

import java.util.Optional;

import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.simulation.LinearRegression;
import frc.robot.simulation.PiSimulation.TimeDistance;

public class CornerDetector {
  private static final double MIN_DISTANCE = 50;
  private static final double MAX_DISTANCE = 1000;

  public record CornerInfo(double cornerTimestamp, double angle) {
  }

  private double slopeThreshold;
  private int largeWindow;
  private int smallWindow;
  private double flatSlopeThreshold;

  private int startIndex = 0;
  private int endIndex = 0;
  private Optional<Double> cornerTimestamp = Optional.empty();
  public double cornerDist = 0;
  
  private CircularBuffer<TimeDistance> data = new CircularBuffer<>(25);

  public CornerDetector(double slopeThreshold, int largeWindow, int smallWindow, double flatSlopeThreshold) {
    this.slopeThreshold = slopeThreshold;
    this.largeWindow = largeWindow;
    this.smallWindow = smallWindow;
    this.flatSlopeThreshold = flatSlopeThreshold;

    this.reset();
  }

  public void reset() {
    startIndex = 0;
    endIndex = 0;
    cornerTimestamp = Optional.empty();
    cornerDist = 0;
    data.clear();
  }

  public boolean foundCorner() {
    return cornerTimestamp.isPresent();
  }

  private boolean isLineLike(LinearRegression regression) {
    final int flatSlopeThreshold = 60;
    return regression.r2value > 0.8 || regression.slope < flatSlopeThreshold;
    // Possibly using RMSE is better so we can get a fit for flat slopes?
    // return regression.r2value > 0.8 || (regression.rmse / 600) < 0.05;
  }

  public double getCornerTimestamp() {
    if (cornerTimestamp.isPresent()) {
      return cornerTimestamp.get();
    }
    // Should not happen, we check first.
    return 0.0;
  }

  public double getCornerDistance() {
    return cornerDist;
  }

  public void addRecord(double timestamp, double distanceMm) {
    if (distanceMm > MAX_DISTANCE) {
      this.reset();
      return;
    }

    if (distanceMm < MIN_DISTANCE) {
      return;
    }

    data.addLast(new TimeDistance((float)Timer.getFPGATimestamp(), (float)distanceMm));
    if (data.size() < largeWindow) {
      return;
    }

    var firstRegression = new LinearRegression(data, 0, smallWindow);
    var secondRegression = new LinearRegression(data, data.size() - smallWindow, data.size());
    if (isLineLike(firstRegression) && isLineLike(secondRegression) &&
        secondRegression.slope - firstRegression.slope > slopeThreshold) {
      if (Math.abs(firstRegression.slope - secondRegression.slope) < 1e-6) {
        return;
      }

      double cornerTimestamp = (secondRegression.intercept - firstRegression.intercept)
          / (firstRegression.slope - secondRegression.slope);
      double cornerDistanceMm = firstRegression.slope * cornerTimestamp + firstRegression.intercept;
      this.cornerTimestamp = Optional.of(cornerTimestamp);
      this.cornerDist = cornerDistanceMm;
      // Convert m/s to mm/s since slope has y-values in mm.
      //double angle = Math.atan2(secondRegression.slope, speed * 1000);
      // return Optional.of(cornerInfo);
    }
  }
}