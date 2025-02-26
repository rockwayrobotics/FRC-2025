package frc.robot;

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
          return 0.55;
        case FRONT_RIGHT:
          return 0.55;
        case BACK_LEFT:
          return 0.55;
        case BACK_RIGHT:
          return 0.55;
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
          return 0.55;
        case FRONT_RIGHT:
          return 0.55;
        case BACK_LEFT:
          return 0.55;
        case BACK_RIGHT:
          return 0.55;
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
  private double cornerDistance = 0;

  public void reset() {
    sensorState = SensorState.NONE;
    reefBar = ReefBar.NEAR;
    reefHeight = ReefHeight.L2;
    cornerDistance = 0;
  }

  public void setCornerDistance(double cornerDistance) {
    this.cornerDistance = cornerDistance;
  }

  public double pivotRadians() {
    switch (sensorState) {
      case FRONT_LEFT:
      case BACK_LEFT:
        return reefHeight.pivotRadians();
      case FRONT_RIGHT:
      case BACK_RIGHT:
        return -reefHeight.pivotRadians();
      default:
        // FIXME: This feels vaguely unsafe
        return 0;
    }
  }

  /**
   * Based on the current encoder position and the stored corner position,
   * using the sensor state and target bar, return true if we should shoot now.
   * @param leftPositionMeters
   * @return
   */
  public boolean readyToShoot(double leftPositionMeters) {
    switch (sensorState) {
      case FRONT_LEFT:
      case FRONT_RIGHT:
        switch (reefBar) {
          case NEAR:
            return sensorState.nearDistance() <= leftPositionMeters - cornerDistance;
          case FAR:
            return sensorState.farDistance() <= leftPositionMeters - cornerDistance;
        }
      case BACK_LEFT:
      case BACK_RIGHT:
        switch (reefBar) {
          case NEAR:
            return sensorState.nearDistance() >= leftPositionMeters - cornerDistance;
          case FAR:
            return sensorState.farDistance() >= leftPositionMeters - cornerDistance;
        }
      default:
        return false;
    }
  }
}
