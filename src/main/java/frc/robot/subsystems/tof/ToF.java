package frc.robot.subsystems.tof;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotTracker;

public class ToF extends SubsystemBase{
  public final ToFIO io;
  private final ToFIOInputsAutoLogged inputs = new ToFIOInputsAutoLogged();

  public ToF(ToFIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("ToF", inputs);
    RobotTracker.getInstance().recordToF(inputs.frontLeft, inputs.backLeft, inputs.frontRight, inputs.backRight);
  }
}
