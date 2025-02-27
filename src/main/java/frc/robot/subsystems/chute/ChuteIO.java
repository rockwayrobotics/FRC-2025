package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.AutoLog;

public interface ChuteIO {
  @AutoLog
  public static class CoralIOInputs {
    // TODO: Figure out proper coordinate system
    public double pivotAngleRadians = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotSetpoint = 0.0;
    public boolean coralLoading = false;
    public boolean coralReady = false;
    public double shooterVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(CoralIOInputs inputs) {}

  public default void setPivotGoal(double pivotAngleRadians) {}

  /**
   * Set the shooter to a normalized speed in [-1, 1].
   * FIXME: This is probably not what we want
   * @param speed
   */
  public default void setShooterSpeed(double speed) {}
}
