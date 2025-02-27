package frc.robot.subsystems.grabber;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.REVUtils;

public class GrabberIOReal implements GrabberIO {
  
  protected final SparkMax leftGrabberMotor = new SparkMax(Constants.CAN.GRABBER_LEFT_MOTOR, MotorType.kBrushless);
  protected final SparkMax rightGrabberMotor = new SparkMax(Constants.CAN.GRABBER_RIGHT_MOTOR, MotorType.kBrushless);
  protected final SparkMax wristMotor = new SparkMax(Constants.CAN.GRABBER_WRIST_MOTOR, MotorType.kBrushless);

  protected final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  protected final DigitalInput homeSwitch = new DigitalInput(Constants.Digital.ALGAE_HOME_SWITCH);
  protected final AnalogInput distanceSensor = new AnalogInput(Constants.Analog.ALGAE_DISTANCE_SENSOR);

  public GrabberIOReal() {
    SparkMaxConfig grabberConfig = new SparkMaxConfig();

    // NEO 550's have a recommended current limit range of 20-40A.
    grabberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);

    grabberConfig.inverted(Constants.Grabber.LEFT_GRABBER_INVERTED);
    REVUtils.tryUntilOk(() -> leftGrabberMotor.configure(grabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    
    grabberConfig.follow(Constants.CAN.GRABBER_LEFT_MOTOR, Constants.Grabber.RIGHT_GRABBER_INVERTED);
    REVUtils.tryUntilOk(() -> rightGrabberMotor.configure(grabberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig wristConfig = new SparkMaxConfig();
    // NEO's have a recommended current limit range of 40-60A, but we deliberately lowered it to 38A for our drive
    // motors. Unclear what this motor limit should be.
    wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    REVUtils.tryUntilOk(() -> wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // FIXME: This is just a sanity value, but we should figure out homing.
    wristEncoder.setPosition(0);
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
