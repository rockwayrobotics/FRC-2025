package frc.robot.subsystems.chute;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ChuteIOSim implements ChuteIO {
  SingleJointedArmSim chuteSim;
  protected final PIDController pivotPid = new PIDController(0.8, 0, 0.1);
  private double pivotVoltage = 0;
  private double shooterVoltage = 0;
  private boolean coralLoading = false;
  private boolean coralReady = false;

  public ChuteIOSim() {
    chuteSim = new SingleJointedArmSim(DCMotor.getNEO(1), Constants.Chute.PIVOT_GEAR_RATIO, Constants.Chute.MOI,
        Constants.Chute.CHUTE_LENGTH_METERS, -Units.degreesToRadians(90), Units.degreesToRadians(90), false, 0.0);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    pivotVoltage = pivotPid.calculate(chuteSim.getAngleRads());
    chuteSim.setInputVoltage(pivotVoltage);

    chuteSim.update(0.02);
    inputs.pivotSetpoint = pivotPid.getSetpoint();
    inputs.coralLoading = coralLoading;
    inputs.coralReady = coralReady;
    inputs.pivotAngleRadians = chuteSim.getAngleRads();
    inputs.pivotVelocityRadPerSec = chuteSim.getVelocityRadPerSec();

    inputs.shooterVelocityRadPerSec = getShooterVelocityRadPerSec();
  }

  @Override
  public void setPivotGoal(double pivotAngleRadians) {
    pivotPid.setSetpoint(pivotAngleRadians);
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterVoltage = voltage;
  }

  public void setCoralLoading(boolean loading) {
    coralLoading = loading;
  }

  public void setCoralReady(boolean ready) {
    coralLoading = ready;
  }

  public double getPivotAngleRads() {
    return chuteSim.getAngleRads();
  }

  public double getShooterVelocityRadPerSec() {
    // FIXME: This is ridiculous, but not sure how to model this.
    return shooterVoltage;
  }
}
