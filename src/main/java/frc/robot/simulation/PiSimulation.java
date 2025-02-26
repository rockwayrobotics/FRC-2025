package frc.robot.simulation;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.ScoringState;
import frc.robot.simulation.WorldSimulation.FieldObstacles;

public class PiSimulation {
  public static final long DEFAULT_STATE = 0;
  private final IntegerSubscriber piStateSubscriber;
  private final FloatPublisher cornerPublisher;
  private final FieldObstacles fieldObstacles;
  private long piState = DEFAULT_STATE;

  public record TimeDistance(float time, float distance) {}

  private CircularBuffer<TimeDistance> data = new CircularBuffer<>(25);

  public PiSimulation(FieldObstacles fieldObstacles) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    piStateSubscriber = nt.getIntegerTopic(Constants.NT.SENSOR_MODE).subscribe(DEFAULT_STATE);
    cornerPublisher = nt.getFloatTopic(Constants.NT.CORNERS).publish();
    this.fieldObstacles = fieldObstacles;
  }

  private Optional<Double> addRecord(float distanceMm) {
    // Basically corner_detector.py in Java
    final int largeWindow = 25;
    final int smallWindow = 10;
    final int flatSlopeThreshold = 60;
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

    data.addLast(new TimeDistance((float)Timer.getFPGATimestamp(), distanceMm));
    if (data.size() < largeWindow) {
      return Optional.empty();
    }

    var firstRegression = new LinearRegression(data, 0, smallWindow);
    var secondRegression = new LinearRegression(data, data.size() - smallWindow, data.size());
    if (
      (firstRegression.r2value > 0.8 || Math.abs(firstRegression.slope) < flatSlopeThreshold) &&
      (secondRegression.r2value > 0.8 || Math.abs(secondRegression.slope) < flatSlopeThreshold) &&
      secondRegression.slope - firstRegression.slope > slopeThreshold
    ) {
      if (Math.abs(firstRegression.slope - secondRegression.slope) < 1e-6) {
        return Optional.empty();
      }
      return Optional.of((secondRegression.intercept - firstRegression.intercept) / (firstRegression.slope - secondRegression.slope));
    }

    return Optional.empty();
  }

  public void periodic(Pose2d robotPose) {
    long[] piStateValues = piStateSubscriber.readQueueValues();
    if (piStateValues.length > 0) {
      piState = piStateValues[piStateValues.length - 1];
    }

    if (piState == ScoringState.SensorState.NONE.piValue()) {
      return;
    }

    Transform2d sensorTransform = null;
    if (piState == ScoringState.SensorState.FRONT_LEFT.piValue()) {
      sensorTransform = Constants.ToFSensor.FRONT_LEFT;
    } else if (piState == ScoringState.SensorState.FRONT_RIGHT.piValue()) {
      sensorTransform = Constants.ToFSensor.FRONT_RIGHT;
    } else if (piState == ScoringState.SensorState.BACK_LEFT.piValue()) {
      sensorTransform = Constants.ToFSensor.BACK_LEFT;
    } else if (piState == ScoringState.SensorState.BACK_RIGHT.piValue()) {
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
      var maybeCornerTimestamp = this.addRecord((float)distanceMm);
      maybeCornerTimestamp.ifPresent(cornerTimestamp -> {
        cornerPublisher.set(cornerTimestamp.floatValue());
        data.clear();
      });
    }
  }
}
