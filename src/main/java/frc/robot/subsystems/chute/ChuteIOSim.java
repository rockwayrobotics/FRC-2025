package frc.robot.subsystems.chute;

import java.util.concurrent.CompletableFuture;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ChuteIOSim implements ChuteIO {
  protected final SingleJointedArmSim chuteSim;
  protected final PIDController pivotPid = new PIDController(0.8, 0, 0.1);
  private double pivotVoltage = 0;

  public ChuteIOSim() {
    chuteSim = new SingleJointedArmSim(DCMotor.getNEO(1), Constants.Chute.PIVOT_GEAR_RATIO, Constants.Chute.MOI,
        Constants.Chute.CHUTE_LENGTH_METERS, Constants.Chute.PIVOT_FLAT, -Constants.Chute.PIVOT_FLAT, false,
        Constants.Chute.PIVOT_INITIAL_ANGLE_RADS);
    pivotPid.setSetpoint(Constants.Chute.PIVOT_INITIAL_ANGLE_RADS);
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    pivotVoltage = pivotPid.calculate(chuteSim.getAngleRads());
    chuteSim.setInputVoltage(pivotVoltage);
    chuteSim.update(0.02);

    inputs.pivotSetpoint = pivotPid.getSetpoint();
    inputs.pivotAngleRadians = chuteSim.getAngleRads();
    inputs.pivotVelocityRadPerSec = chuteSim.getVelocityRadPerSec();
  }

  @Override
  public void moveTowardsPivotGoal(double pivotAngleRadians, double currentAngleRadians) {
    pivotPid.setSetpoint(pivotAngleRadians);
  }

  @Override
  public void stopPivot() {
    // FIXME
  }


  public double getPivotAngleRads() {
    return chuteSim.getAngleRads();
  }

  public CompletableFuture<Boolean> home() {
    var promise = new CompletableFuture<Boolean>();
    promise.complete(true);
    return promise;
  }
}
