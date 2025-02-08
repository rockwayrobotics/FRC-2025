package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  private final GrabberIO io;
  private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

  public Grabber(GrabberIO io) {
    this.io = io;
  }
    
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Grabber", inputs);
  }
}
