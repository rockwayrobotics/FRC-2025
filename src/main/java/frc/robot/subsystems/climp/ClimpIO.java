package frc.robot.subsystems.climp;

import org.littletonrobotics.junction.AutoLog;

public interface ClimpIO {
  @AutoLog
  public static class ClimpIOInputs {
    public double appliedVoltage = 0;
    public double supplyCurrentAmps = 0;
    public double angleRadians = 0;
    public double velocityRadsPerSec = 0;
  }
    
  public default void updateInputs(ClimpIOInputs inputs) {}

  public default void setNormalizedSpeed(double speed) {}

  public default void stop() {}

  public default void moveTowardsGoal(double goalAngleRadians, double currentAngleRadians) {}
}
