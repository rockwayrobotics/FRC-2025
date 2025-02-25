package frc.robot.subsystems.climp;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;
import frc.robot.util.REVUtils;

public class ClimpIOReal implements ClimpIO {
  protected final SparkFlex climpMotor = new SparkFlex(Constants.CAN.CLIMP_MOTOR, MotorType.kBrushless);

  protected final RelativeEncoder encoder = climpMotor.getEncoder();
  
  public ClimpIOReal() {
    SparkFlexConfig config = new SparkFlexConfig();
    // Vortex default smart current limit is 80 A.
    // https://docs.revrobotics.com/brushless/faq#what-smart-current-limit-should-i-set-for-my-neo-vortex
    config.idleMode(IdleMode.kBrake).voltageCompensation(12.0);
    // FIXME: Set encoder conversion factor in order to convert to angle based on gear reduction
    // config.encoder.positionConversionFactor(1.0 / GEAR_REDUCTION);

    REVUtils.tryUntilOk(() -> climpMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }  

  @Override
  public void updateInputs(ClimpIOInputs inputs) {
    REVUtils.ifOk(climpMotor, new DoubleSupplier[] {
        climpMotor::getAppliedOutput, climpMotor::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(climpMotor, climpMotor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
  }

  @Override
  public void setNormalizedSpeed(double speed) {
    climpMotor.set(speed);
  }
}
