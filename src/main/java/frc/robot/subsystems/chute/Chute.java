package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.util.Interlock;
import frc.robot.util.Tuner;

public class Chute {
  private final ChuteIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  final Interlock unlocked = new Interlock("Chute");
  final Tuner shooterSpeedTuner = new Tuner("ShooterSpeed", 0.3, true);

  private double pivotGoalRads = Constants.Chute.PIVOT_INITIAL_ANGLE_RADS;
  private double pivotAngleRads = Constants.Chute.PIVOT_INITIAL_ANGLE_RADS;
  private boolean coralLoading = false;
  private boolean coralReady = false;
  private boolean isHomed = false;

  public Chute(ChuteIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Chute", inputs);
    coralLoading = inputs.coralLoading;
    coralReady = inputs.coralReady;
    pivotAngleRads = inputs.pivotAngleRadians;

    if (DriverStation.isDisabled() || !unlocked.get() || !isHomed) {
      io.stopPivot();
      io.setShooterSpeed(0);
    } else {
      io.moveTowardsPivotGoal(pivotGoalRads, inputs.pivotAngleRadians);
    }
  }

  public void setPivotGoalRads(double pivotAngleRads) {
    if (unlocked.get()) {
      pivotGoalRads = pivotAngleRads;
    }
  }

  public double getPivotGoalRads() {
    return pivotGoalRads;
  }

  public double getPivotAngleRads() {
    return pivotAngleRads;
  }

  public void startShooting() {
    io.setShooterSpeed(shooterSpeedTuner.get());
  }

  public void setShooterMotor(double speed) {
    io.setShooterSpeed(speed);
  }

  public void stopShooting() {
    io.setShooterSpeed(0);
  }

  public boolean isCoralLoading() {
    return coralLoading;
  }

  public boolean isCoralReady() {
    return coralReady;
  }

  public void setBrakeMode(boolean mode) {
    io.setBrakeMode(mode);
  }

  public void home() {
    var promise = io.home();
    System.out.println("home starting");
    Logger.recordOutput("Chute/Homing", true);
    Commands.waitUntil(() -> promise.isDone()).finallyDo(() -> {
      this.isHomed = promise.getNow(false);
      Logger.recordOutput("Chute/Homing", false);
      Logger.recordOutput("Chute/Homed_real", this.isHomed);
      System.out.println("home finished");
    }).schedule();
  }

  public boolean isHomed() {
    return isHomed;
  }
}
