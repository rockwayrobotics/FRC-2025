package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
  @AutoLog
  public static class GrabberIOInputs {
    public double wristAngleRadians = 0;
    public double wristVelocityRadPerSec = 0;
    public boolean home = false;
    public double wristAppliedVolts = 0;
  }

  public default void updateInputs(GrabberIOInputs inputs) {}

  public default void setWristMotor(double speed) {}

  public default void setGrabberMotor(double speed) {}

  public default void setBrakeMode(boolean mode) {}

  public default void stopWrist() {}

  public default void moveTowardsGoal(double goalAngleRadians, double currentAngleRadians) {}

  public default void home() {}

  // public default double getAlgaeDistance() {return 0;}

  // public default boolean atHome() {return false;}

}

