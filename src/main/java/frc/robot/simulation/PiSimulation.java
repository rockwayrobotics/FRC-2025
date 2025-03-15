package frc.robot.simulation;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.simulation.WorldSimulation.FieldObstacles;

public class PiSimulation {
  public static final String DEFAULT_STATE = "none";
  private final StringSubscriber modeSubscriber;
  private final FloatArrayPublisher cornerPublisher;
  private final FieldObstacles fieldObstacles;
  private String piMode = DEFAULT_STATE;

  public record TimeDistance(float time, float distance) {
  }

  public record CornerInfo(double cornerTimestamp, double angle) {
  }

  private CircularBuffer<TimeDistance> data = new CircularBuffer<>(25);

  public PiSimulation(FieldObstacles fieldObstacles) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    modeSubscriber = nt.getStringTopic(Constants.NT.TOF_MODE).subscribe("none");
    cornerPublisher = nt.getFloatArrayTopic(Constants.NT.CORNERS).publish();
    this.fieldObstacles = fieldObstacles;
  }

  private boolean isLineLike(LinearRegression regression) {
    final int flatSlopeThreshold = 60;
    return regression.r2value > 0.8 || regression.slope < flatSlopeThreshold;
    // Possibly using RMSE is better so we can get a fit for flat slopes?
    // return regression.r2value > 0.8 || (regression.rmse / 600) < 0.05;
  }

  private Optional<CornerInfo> addRecord(float distanceMm, double speed) {
    // Basically corner_detector.py in Java
    final int largeWindow = 25;
    final int smallWindow = 10;
    final double slopeThreshold = 400;
    final float minDistance = 150;
    final float maxDistance = 750;

    if (distanceMm > maxDistance) {
      data.clear();
      return Optional.empty();
    }

    if (distanceMm < minDistance) {
      return Optional.empty();
    }

    data.addLast(new TimeDistance((float) Timer.getFPGATimestamp(), distanceMm));
    if (data.size() < largeWindow) {
      return Optional.empty();
    }

    var firstRegression = new LinearRegression(data, 0, smallWindow);
    var secondRegression = new LinearRegression(data, data.size() - smallWindow, data.size());
    if (isLineLike(firstRegression) && isLineLike(secondRegression) &&
        secondRegression.slope - firstRegression.slope > slopeThreshold) {
      if (Math.abs(firstRegression.slope - secondRegression.slope) < 1e-6) {
        return Optional.empty();
      }

      double cornerTimestamp = (secondRegression.intercept - firstRegression.intercept)
          / (firstRegression.slope - secondRegression.slope);
      // Convert m/s to mm/s since slope has y-values in mm.
      double angle = Math.atan2(secondRegression.slope, speed * 1000);
      CornerInfo cornerInfo = new CornerInfo(cornerTimestamp, angle);
      return Optional.of(cornerInfo);
    }

    return Optional.empty();
  }

  public void periodic(Pose2d robotPose) {
    String[] modes = modeSubscriber.readQueueValues();
    if (modes.length > 0) {
      piMode = modes[modes.length - 1];
    }

    if (piMode.equals("none")) {
      return;
    }

    Transform2d sensorTransform = null;
    if (piMode.equals("left") || piMode.equals("front-left")) {
      sensorTransform = Constants.ToFSensor.FRONT_LEFT;
    } else if (piMode.equals("right") || piMode.equals("front-right")) {
      sensorTransform = Constants.ToFSensor.FRONT_RIGHT;
    } else if (piMode.equals("back-left")) {
      sensorTransform = Constants.ToFSensor.BACK_LEFT;
    } else if (piMode.equals("back-right")) {
      sensorTransform = Constants.ToFSensor.BACK_RIGHT;
    }

    if (sensorTransform == null) {
      return;
    }

    Translation2d sensorPose = robotPose
        .transformBy(new Transform2d(sensorTransform.getTranslation(), robotPose.getRotation()))
        .getTranslation();
    double angle = robotPose.getRotation().getRadians()
        + sensorTransform.getRotation().getRadians();
    double distanceMm = ToFSimUtils.simulateSensor(sensorPose, angle, fieldObstacles);
    if (distanceMm < Double.POSITIVE_INFINITY) {
      var maybeCornerInfo = this.addRecord((float) distanceMm, 0.45);
      maybeCornerInfo.ifPresent(cornerInfo -> {
        cornerPublisher.set(new float[] { (float) cornerInfo.cornerTimestamp, (float) cornerInfo.angle });
        data.clear();
      });
    }
  }
}
