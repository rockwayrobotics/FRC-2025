package frc.robot.subsystems.climp;

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
  
  public ClimpIOReal() {
    SparkFlexConfig config = new SparkFlexConfig();
    config.idleMode(IdleMode.kBrake);
    
    REVUtils.tryUntilOk(() -> climpMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }  

  @Override
  public void updateInputs(ClimpIOInputs inputs) {
    
  }

  @Override
  public void setClimpMotor(double speed) {
    climpMotor.set(speed);
  }
}
