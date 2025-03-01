package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Interlock;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeightMillimeters = 0;
  private double heightMillimeters = 0;
  private boolean homed = false;

  final Interlock enabled = new Interlock("Elevator");

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    heightMillimeters = inputs.positionMillimeters;
    homed = inputs.homed;

    if (DriverStation.isDisabled() || !enabled.get()) {
      io.stop();
    } else {
      io.moveTowardsGoal(goalHeightMillimeters, heightMillimeters);
    }
  }

  public void setGoalHeightMillimeters(double heightMillimeters) {
    if (enabled.get()) {
      goalHeightMillimeters = heightMillimeters;
    }
  }

  public double getHeightMillimeters() {
    return heightMillimeters;
  }

  public double getGoalHeightMillimeters() {
    return goalHeightMillimeters;
  }

  public boolean atGoal() {
    // within 1 centimeter
    return Math.abs(heightMillimeters - goalHeightMillimeters) < 10;
  }

  public void stop() {
    io.stop();
  }

  public boolean isHomed() {
    return homed;
  }
}
