package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.AutoLog;

public interface ChuteIO {
  @AutoLog
  public static class CoralIOInputs {
    // TODO: Figure out proper coordinate system
    public double pivotAngleRadians = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotSetpoint = 0.0;
    public boolean coralLoaded = false;
    public double shooterVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(CoralIOInputs inputs) {}

  public default void setPivotGoal(double pivotAngleRadians) {}

  // FIXME: This is probably not what we want
  public default void setShooterVoltage(double voltage) {}
}
