package frc.robot.subsystems.chute;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ChuteIOSim implements ChuteIO {
  protected final SingleJointedArmSim chuteSim;
  protected final FlywheelSim shooterSim;
  protected final PIDController pivotPid = new PIDController(0.8, 0, 0.1);
  private double pivotVoltage = 0;
  private double shooterSpeed = 0;
  private boolean coralLoading = false;
  private boolean coralReady = false;

  public ChuteIOSim() {
    chuteSim = new SingleJointedArmSim(DCMotor.getNEO(1), Constants.Chute.PIVOT_GEAR_RATIO, Constants.Chute.MOI,
        Constants.Chute.CHUTE_LENGTH_METERS, Constants.Chute.PIVOT_FLAT, -Constants.Chute.PIVOT_FLAT, false,
        Constants.Chute.PIVOT_INITIAL_ANGLE_RADS);
    // Not sure if this is actually a flywheel, and these are random numbers
    shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.2, 10 / 18.0),
        DCMotor.getNEO(1));
    pivotPid.setSetpoint(Constants.Chute.PIVOT_INITIAL_ANGLE_RADS);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    pivotVoltage = pivotPid.calculate(chuteSim.getAngleRads());
    chuteSim.setInputVoltage(pivotVoltage);
    chuteSim.update(0.02);

    shooterSim.setInputVoltage(12.0 * shooterSpeed);
    shooterSim.update(0.02);
    shooterSim.getAngularVelocityRadPerSec();

    inputs.pivotSetpoint = pivotPid.getSetpoint();
    inputs.coralLoading = coralLoading;
    inputs.coralReady = coralReady;
    inputs.pivotAngleRadians = chuteSim.getAngleRads();
    inputs.pivotVelocityRadPerSec = chuteSim.getVelocityRadPerSec();

    inputs.shooterVelocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
  }

  @Override
  public void setPivotGoal(double pivotAngleRadians) {
    pivotPid.setSetpoint(pivotAngleRadians);
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterSpeed = speed;
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
    return shooterSim.getAngularVelocityRadPerSec();
  }
}
