package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeightMeters = 0;
  private double heightMeters = 0;
    
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    heightMeters = inputs.positionMeters;

    if (DriverStation.isDisabled()) {
      io.stop();
    }
  }

  public void setGoalHeightMeters(double heightMeters) {
    goalHeightMeters = heightMeters;
    io.setGoal(goalHeightMeters);
  }

  public boolean atGoal() {
    return heightMeters == goalHeightMeters;
  }

  public void stop() {
    io.stop();
  }
}
