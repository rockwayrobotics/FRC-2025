package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Interlock;
import frc.robot.util.Sensors;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeightMillimeters = 0;
  private double heightMillimeters = 0;
  private boolean homeBeamBroken = false;
  private boolean isHomed = false;

  final Interlock unlocked = new Interlock("Elevator");

  public Elevator(ElevatorIO io) {
    this.io = io;
    unlocked.addListener((e) -> {
      if (e.valueData.value.getBoolean()) {
        setGoalHeightMillimeters(heightMillimeters);
      }
    });
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    heightMillimeters = inputs.positionMillimeters;
    homeBeamBroken = inputs.homeBeamBroken;
    Logger.recordOutput("Elevator/goal_height_mm", goalHeightMillimeters);

    if (DriverStation.isDisabled() || !isHomed) {
      io.stop();
    } else {
      io.moveTowardsGoal(goalHeightMillimeters, heightMillimeters);
    }
  }

  public void setGoalHeightMillimeters(double heightMillimeters) {
    goalHeightMillimeters = heightMillimeters;
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

  public boolean isHomeBeamBroken() {
    return homeBeamBroken;
  }

  public void home() {
    io.zeroEncoder();
    setGoalHeightMillimeters(0);
    isHomed = true;
    // if (!Sensors.getInstance().getElevatorHomeBeambroken()) {
    // io.zeroEncoder();
    // isHomed = true;
    // homeBeamBroken = true;
    // } else {
    // System.err.println("Elevator home beam is NOT broken. It is NOT homed.");
    // }
  }

  public void stayStill() {
    setGoalHeightMillimeters(heightMillimeters);
  }
}
