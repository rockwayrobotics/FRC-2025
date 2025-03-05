package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Interlock;

public class Grabber {
  private final GrabberIO io;
  private final GrabberIOInputsAutoLogged inputs = new GrabberIOInputsAutoLogged();

  final Interlock unlocked = new Interlock("Grabber");
  private Boolean isHomed = false;

  private double wristGoalRads = 0;

  public Grabber(GrabberIO io) {
    this.io = io;
  }

  public void setBrakeMode(boolean mode) {
    io.setBrakeMode(mode);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Grabber", inputs);

    if (DriverStation.isDisabled() || !unlocked.get()) {
      io.stopWrist();
    } else {
      // right now positive wrist speed is up, negative is down
      io.moveTowardsGoal(wristGoalRads, inputs.wristAngleRadians);
    }

  }

  public void setWristGoalRads(double wristAngleRads) {
    // right now positive wrist speed is up, negative is down
    if (unlocked.get()) {
      wristGoalRads = wristAngleRads;
    }
  }

  public double getCurrentRads() {
    return inputs.wristAngleRadians;
  }

  public double getWristGoalRads() {
    return wristGoalRads;
  }

  public void home() {
    io.home();
  }
}
