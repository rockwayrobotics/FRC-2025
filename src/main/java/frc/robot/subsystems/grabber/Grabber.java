package frc.robot.subsystems.grabber;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
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
    unlocked.addListener((e) -> {
      if (e.valueData.value.getBoolean()) {
        wristGoalRads = getCurrentRads();
      }
    });
  }

  public void setBrakeMode(boolean mode) {
    io.setBrakeMode(mode);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Grabber", inputs);
    Logger.recordOutput("Grabber/goal_radian", wristGoalRads);
    Logger.recordOutput("Grabber/isHomed", isHomed);

    if (DriverStation.isDisabled() || !isHomed) { // FIXME: add homed check when we have a homing sequence
      io.stopWrist();
    } else {
      // right now negative wrist speed is up, positive is down
      io.moveTowardsGoal(wristGoalRads, inputs.wristAngleRadians);
    }

  }

  public void setWristGoalRads(double wristAngleRads) {
    // right now negative wrist speed is up, positive is down
    wristGoalRads = wristAngleRads;
  }

  public double getCurrentRads() {
    return inputs.wristAngleRadians;
  }

  public double getWristGoalRads() {
    return wristGoalRads;
  }

  public void setGrabberMotor(double speed) {
    io.setGrabberMotor(speed);
  }

  public void home() {
    io.home();
    setWristGoalRads(Units.degreesToRadians(-90));
    isHomed = true;
  }

  public void stayStill() {
    setWristGoalRads(inputs.wristAngleRadians);
    setGrabberMotor(0);
  }
}
