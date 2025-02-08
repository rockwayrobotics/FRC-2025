package frc.robot.subsystems.chute;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ChuteIOSim implements ChuteIO {
  SingleJointedArmSim chuteSim;
  protected final PIDController pivotPid = new PIDController(0.5, 0, 0);
  private double pivotVoltage = 0;
  private double shooterVoltage = 0;
  private boolean coralLoaded = false;

  public ChuteIOSim() {
    chuteSim = new SingleJointedArmSim(DCMotor.getNEO(1), Constants.Coral.PIVOT_GEAR_RATIO, Constants.Coral.MOI,
        Constants.Coral.CHUTE_LENGTH_METERS, -Units.degreesToRadians(90), Units.degreesToRadians(90), false, 0.0, 0.0);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    pivotVoltage = pivotPid.calculate(chuteSim.getVelocityRadPerSec());
    chuteSim.setInputVoltage(pivotVoltage);

    chuteSim.update(0.02);
    inputs.coralLoaded = coralLoaded;
    inputs.pivotAngleRadians = chuteSim.getAngleRads();
    inputs.pivotVelocityRadPerSec = chuteSim.getVelocityRadPerSec();

    // FIXME: This is ridiculous, but not sure how to model this.
    inputs.shooterVelocityRadPerSec = shooterVoltage;
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
}
