package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.REVUtils;

public class GrabberIOReal implements GrabberIO {
  
  protected final SparkMax leftGrabberMotor = new SparkMax(Constants.CAN.GRABBER_LEFT_MOTOR, MotorType.kBrushless);
  protected final SparkMax rightGrabberMotor = new SparkMax(Constants.CAN.GRABBER_RIGHT_MOTOR, MotorType.kBrushless);
  protected final SparkMax wristMotor = new SparkMax(Constants.CAN.GRABBER_WRIST_MOTOR, MotorType.kBrushless);

  protected final DigitalInput homeSwitch = new DigitalInput(Constants.Digital.ALGAE_HOME_SWITCH);
  protected final AnalogInput distanceSensor = new AnalogInput(Constants.Analog.ALGAE_DISTANCE_SENSOR);

  public GrabberIOReal() {
    SparkMaxConfig config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);

    config.inverted(Constants.Grabber.LEFT_GRABBER_INVERTED);
    REVUtils.tryUntilOk(() -> leftGrabberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    config.inverted(Constants.Grabber.RIGHT_GRABBER_INVERTED);
    config.follow(Constants.CAN.GRABBER_LEFT_MOTOR);
    REVUtils.tryUntilOk(() -> rightGrabberMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override 
  public void setWristMotor(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public void setGrabberMotor(double speed) {
    leftGrabberMotor.set(speed);
  }

  @Override
  public void updateInputs(GrabberIOInputs inputs) {
    inputs.algaeDistance = this.distanceSensor.getVoltage();
    inputs.home = this.homeSwitch.get(); 
  }
}
