package frc.robot.subsystems.climp;

import org.littletonrobotics.junction.AutoLog;

public interface ClimpIO {
  @AutoLog
  public static class ClimpIOInputs {
    public double appliedVoltage = 0;
    public double supplyCurrentAmps = 0;
    public double positionMeters = 0;
    public double velocityMetersPerSec = 0;
  }
    
  public default void updateInputs(ClimpIOInputs inputs) {}

  public default void setNormalizedSpeed(double speed) {}

  public default void setClimpMotor(double speed) {}
}
