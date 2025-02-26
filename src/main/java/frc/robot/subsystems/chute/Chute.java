package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chute extends SubsystemBase {
  private final ChuteIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  private double pivotGoalRads = 0.0;
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
    io.setShooterVoltage(1);
  }

  public void stopShooting() {
    io.setShooterVoltage(0);
  }

  public boolean isCoralLoading() {
    return coralLoading;
  }

  public boolean isCoralReady() {
    return coralReady;
  }
}
