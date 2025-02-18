package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;

public class TempBeamBreak {
  private DigitalInput input = new DigitalInput(2);
  private AsynchronousInterrupt interrupt;
  private double lastTimestamp = 0;

  public TempBeamBreak() {
    interrupt = new AsynchronousInterrupt(input, this::callback);
    interrupt.setInterruptEdges(false, true);
    interrupt.enable();
  }

  public void periodic() {
    var value = input.get();
    Logger.recordOutput("DigitalIO/PeriodicValue", value);
    if (lastTimestamp > 0) {
      Logger.recordOutput("DigitalIO/Falling", lastTimestamp);
      lastTimestamp = 0;
    }
  }

  public void enable() {
    interrupt.enable();
  }

  public void disable() {
    interrupt.disable();
  }

  public void callback(boolean rising, boolean falling) {
    if (falling) {
      lastTimestamp = interrupt.getFallingTimestamp();
    }
  }
}
