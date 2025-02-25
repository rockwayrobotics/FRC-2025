package frc.robot.subsystems.chute;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Chute extends SubsystemBase {
  private final ChuteIO io;
  private final CoralIOInputsAutoLogged inputs = new CoralIOInputsAutoLogged();

  private double pivotGoalRads = 0.0;
  
  public Chute(ChuteIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Chute", inputs);
  }

  public void setPivotGoalRads(double pivotAngleRads) {
    pivotGoalRads = pivotAngleRads;
    io.setPivotGoal(pivotGoalRads);
  }

  public double getPivotGoalRads() {
    return pivotGoalRads;
  }

  public void shoot() {
    io.setShooterVoltage(1);
  }
}
