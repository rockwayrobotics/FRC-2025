package frc.robot.subsystems.climp;

import org.littletonrobotics.junction.AutoLog;

public interface ClimpIO {
  @AutoLog
  public static class ClimpIOInputs {
    
  }
    
  public default void updateInputs(ClimpIOInputs inputs) {}
}
