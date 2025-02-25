package frc.robot.subsystems.climp;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
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
    config.idleMode(IdleMode.kBrake);
    
    REVUtils.tryUntilOk(() -> climpMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }  

  @Override
  public void updateInputs(ClimpIOInputs inputs) {
    REVUtils.ifOk(climpMotor, encoder::getPosition, (value) -> inputs.positionMeters = value);
    REVUtils.ifOk(climpMotor, encoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    REVUtils.ifOk(climpMotor, new DoubleSupplier[] {
        climpMotor::getAppliedOutput, climpMotor::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(climpMotor, climpMotor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);

  }

  @Override
  public void setClimpMotor(double speed) {
    climpMotor.set(speed);
  }
}
