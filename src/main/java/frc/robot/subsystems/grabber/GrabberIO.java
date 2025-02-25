package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
  @AutoLog
  public static class GrabberIOInputs {
    public double algaeDistance = 0;
    public boolean home = false;
  }

  public default void updateInputs(GrabberIOInputs inputs) {}

  public default void setWristMotor(double speed) {}

  public default void setGrabberMotor(double speed) {}

  // public default double getAlgaeDistance() {return 0;}

  // public default boolean atHome() {return false;}

}

