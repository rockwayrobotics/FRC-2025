package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean homed = false;
    // TODO: Figure out how to report faults / connectivity?
    // public boolean motorConnected = true;
    public double positionMillimeters = 0.0;
    public double velocityMillimetersPerSec = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double tempCelsius = 0.0;
  }

  /** Read values back from motors */
  public default void updateInputs(ElevatorIOInputs inputs) {
  }

  /**
   * Command a specific height from the elevator. Up is positive, down is
   * negative.
   */
  public default void moveTowardsGoal(double goalHeightMillimeters, double currentHeightMillimeters) {
  }

  public default void stop() {
  }

  public default void periodic() {
  }

  public default void zeroEncoder() {
  }
}
