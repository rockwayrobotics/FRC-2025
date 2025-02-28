package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Tuner;

public class Chute extends SubsystemBase {
  private final ChuteIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  final Tuner shooterSpeedTuner = new Tuner("ShooterSpeed", 0.3, true);

  private double pivotGoalRads = Constants.Chute.PIVOT_INITIAL_ANGLE_RADS;
  private boolean coralLoading = false;
  private boolean coralReady = false;
  
  public Chute(ChuteIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Chute", inputs);
    coralLoading = inputs.coralLoading;
    coralReady = inputs.coralReady;
  }

  public void setPivotGoalRads(double pivotAngleRads) {
    pivotGoalRads = pivotAngleRads;
    io.setPivotGoal(pivotGoalRads);
  }

  public double getPivotGoalRads() {
    return pivotGoalRads;
  }

  public void startShooting() {
    io.setShooterSpeed(shooterSpeedTuner.get());
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
}
