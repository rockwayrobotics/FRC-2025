package frc.robot.subsystems.climp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climp extends SubsystemBase {
  private final ClimpIO io;
  private final ClimpIOInputsAutoLogged inputs = new ClimpIOInputsAutoLogged();

  public Climp(ClimpIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climp", inputs);
  }
}
