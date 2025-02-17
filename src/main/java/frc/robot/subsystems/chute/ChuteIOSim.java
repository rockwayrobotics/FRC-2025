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
  private boolean coralLoaded = false;

  public ChuteIOSim() {
    chuteSim = new SingleJointedArmSim(DCMotor.getNEO(1), Constants.Chute.PIVOT_GEAR_RATIO, Constants.Chute.MOI,
        Constants.Chute.CHUTE_LENGTH_METERS, Units.degreesToRadians(0), Units.degreesToRadians(180), false, 0.0);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    pivotVoltage = pivotPid.calculate(chuteSim.getAngleRads());
    chuteSim.setInputVoltage(pivotVoltage);

    chuteSim.update(0.02);
    inputs.pivotSetpoint = pivotPid.getSetpoint();
    inputs.coralLoaded = coralLoaded;
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

  public void setCoralLoaded(boolean loaded) {
    coralLoaded = loaded;
  }

  public double getPivotAngleRads() {
    return chuteSim.getAngleRads();
  }

  public double getShooterVelocityRadPerSec() {
    // FIXME: This is ridiculous, but not sure how to model this.
    return shooterVoltage;
  }
}
