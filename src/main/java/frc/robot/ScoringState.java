package frc.robot;

import org.littletonrobotics.junction.Logger;

public class ScoringState {
  public enum SensorState {
    NONE(0),
    FRONT_LEFT(1),
    FRONT_RIGHT(2),
    BACK_LEFT(3),
    BACK_RIGHT(4);

    private final int value;

    SensorState(int value) {
      this.value = value;
    }

    public int piValue() {
      return value;
    }

    /**
     * Distance in meters between selected ToF sensor and center of near reef bar,
     * assuming front sensors are used for going forwards and back sensors for
     * scoring while reversing.
     * 
     * @return distance in meters
     */
    public double nearDistance() {
      switch (this) {
        case FRONT_LEFT:
          return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS + Constants.ToFSensor.FRONT_LEFT.getX()
              - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        case FRONT_RIGHT:
          return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS + Constants.ToFSensor.FRONT_RIGHT.getX()
              - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        case BACK_LEFT:
          return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS - Constants.ToFSensor.BACK_LEFT.getX()
              + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        case BACK_RIGHT:
          return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS - Constants.ToFSensor.BACK_RIGHT.getX()
              + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        default:
          return 0;
      }
    }

    /**
     * Distance in meters between selected ToF sensor and center of far reef bar,
     * assuming front sensors are used for going forwards and back sensors for
     * scoring while reversing.
     * 
     * @return distance in meters
     */
    public double farDistance() {
      switch (this) {
        case FRONT_LEFT:
          return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS + Constants.ToFSensor.FRONT_LEFT.getX()
              - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        case FRONT_RIGHT:
          return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS + Constants.ToFSensor.FRONT_RIGHT.getX()
              - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        case BACK_LEFT:
          return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS - Constants.ToFSensor.BACK_LEFT.getX()
              + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        case BACK_RIGHT:
          return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS - Constants.ToFSensor.BACK_RIGHT.getX()
              + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        default:
          return 0;
      }
    }
  }

  public enum ReefBar {
    NEAR,
    FAR
  }

  public enum ReefHeight {
    TROUGH(Constants.Elevator.TROUGH_HEIGHT, Constants.Chute.PIVOT_TROUGH),
    L2(Constants.Elevator.L2_HEIGHT, Constants.Chute.PIVOT_L2),
    L3(Constants.Elevator.L3_HEIGHT, Constants.Chute.PIVOT_L3);

    private final double elevatorHeightMeters;
    private final double pivotRadians;

    ReefHeight(double elevatorHeightMeters, double pivotRadians) {
      this.elevatorHeightMeters = elevatorHeightMeters;
      this.pivotRadians = pivotRadians;
    }

    public double elevatorHeight() {
      return elevatorHeightMeters;
    }

    public double pivotRadians() {
      return pivotRadians;
    }
  }

  public SensorState sensorState = SensorState.NONE;
  public ReefBar reefBar = ReefBar.NEAR;
  public ReefHeight reefHeight = ReefHeight.L2;

  public void reset() {
    sensorState = SensorState.NONE;
    reefBar = ReefBar.NEAR;
    reefHeight = ReefHeight.L2;
  }

  public double pivotRadians() {
    switch (sensorState) {
      case FRONT_LEFT:
      case BACK_LEFT:
        return -reefHeight.pivotRadians();
      case FRONT_RIGHT:
      case BACK_RIGHT:
        return reefHeight.pivotRadians();
      default:
        // FIXME: This feels vaguely unsafe
        return 0;
    }
  }

  public double getTargetWallDistance() {
    int backOrForwards = 1;
    if (sensorState == SensorState.BACK_LEFT || sensorState == SensorState.BACK_RIGHT) {
      backOrForwards = -1;
    }

    switch (reefBar) {
      case FAR:
        return backOrForwards * sensorState.farDistance();
      default:
        return backOrForwards * sensorState.nearDistance();
    }
  }

  public void logOutput() {
    Logger.recordOutput("RobotTracker/ScoringState/SensorState", sensorState);
    Logger.recordOutput("RobotTracker/ScoringState/ReefBar", reefBar);
    Logger.recordOutput("RobotTracker/ScoringState/ReefHeight", reefHeight);
  }
}
