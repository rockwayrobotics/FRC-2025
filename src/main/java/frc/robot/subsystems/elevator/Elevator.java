package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeight = 0;
  private double height = 0;
    
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    height = inputs.positionMeters;

    if (DriverStation.isDisabled()) {
      io.stop();
    }
  }

  public void setGoalHeight(double height) {
    goalHeight = height;
    io.setGoal(goalHeight);
  }

  public boolean atGoal() {
    return height == goalHeight;
  }

  public void stop() {
    io.stop();
  }
}
