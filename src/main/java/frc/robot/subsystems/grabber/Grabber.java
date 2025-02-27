package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Grabber extends SubsystemBase {
  private final GrabberIO io;
  private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();
  
  private double wristSpeed = 0;

  public Grabber(GrabberIO io) {
    this.io = io;
  }
    
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Grabber", inputs);

    // right now positive wrist speed is up, negative is down
    if (inputs.home && wristSpeed > 0) {
      wristSpeed = 0;
    }
    io.setWristMotor(wristSpeed);
  }
}
