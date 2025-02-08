package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
  @AutoLog
  public static class GrabberIOInputs {
  }

  public default void updateInputs(GrabberIOInputs inputs) {}
}
