package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.util.Interlock;

public class Chute {
  private final ChuteIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  final Interlock unlocked = new Interlock("Chute");

  private double pivotGoalRads = Constants.Chute.PIVOT_INITIAL_ANGLE_RADS;
  private double pivotAngleRads = Constants.Chute.PIVOT_INITIAL_ANGLE_RADS;
  private boolean isHomed = false;

  public Chute(ChuteIO io) {
    this.io = io;
  }

  public void periodic(double minAngleRads) {
    io.updateInputs(inputs);
    Logger.processInputs("Chute", inputs);
    Logger.recordOutput("Chute/isHomed", isHomed);

    pivotAngleRads = inputs.pivotAngleRadians;

    double allowedPivotGoalRads = pivotGoalRads;
    if (Math.abs(pivotGoalRads) < minAngleRads) {
      allowedPivotGoalRads = Math.signum(pivotGoalRads) * minAngleRads;
    }

    if (DriverStation.isDisabled() || !isHomed) {
      io.stopPivot();
      pivotGoalRads = allowedPivotGoalRads;
    } else {
      io.moveTowardsPivotGoal(allowedPivotGoalRads, inputs.pivotAngleRadians);
    }
  }

  public void setPivotGoalRads(double pivotAngleRads) {
    pivotGoalRads = pivotAngleRads;
  }

  public double getPivotGoalRads() {
    return pivotGoalRads;
  }

  public double getPivotAngleRads() {
    return pivotAngleRads;
  }

  public boolean isPivotAtGoal() {
    return Math.abs(pivotAngleRads - pivotGoalRads) < Units.degreesToRadians(3);
  }

  public void setBrakeMode(boolean mode) {
    io.setBrakeMode(mode);
  }

  public void home() {
    // var promise = io.home();
    // Commands.waitUntil(() -> promise.isDone()).finallyDo(() -> {
    // this.isHomed = promise.getNow(false);
    // }).schedule();
    io.setEncoder(Units.degreesToRadians(-90));
    this.isHomed = true;
  }

  public void setEncoder(double position) {
    io.setEncoder(position);
  }

  public boolean isHomed() {
    return isHomed;
  }

  public void setIsHomed(boolean v) {
    this.isHomed = v;
  }

  public void stayStill() {
    setPivotGoalRads(pivotAngleRads);
  }
}
