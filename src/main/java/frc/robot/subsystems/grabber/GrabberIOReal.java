package frc.robot.subsystems.grabber;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.util.REVUtils;
import frc.robot.util.Sensors;
import frc.robot.util.Tuner;

public class GrabberIOReal implements GrabberIO {

  protected final SparkMax leftGrabberMotor = new SparkMax(Constants.CAN.GRABBER_LEFT_MOTOR, MotorType.kBrushless);
  protected final SparkMax rightGrabberMotor = new SparkMax(Constants.CAN.GRABBER_RIGHT_MOTOR, MotorType.kBrushless);
  protected final SparkMax wristMotor = new SparkMax(Constants.CAN.GRABBER_WRIST_MOTOR, MotorType.kBrushless);
  protected final SparkClosedLoopController controller = wristMotor.getClosedLoopController();

  final Tuner WristFeedforwardkS = new Tuner("WristFeedforwardkS", 0, true);
  final Tuner WristFeedforwardkG = new Tuner("WristFeedforwardkG", 0, true);
  final Tuner WristPID_P = new Tuner("WristPID_P", 0, true);
  final Tuner WristPID_D = new Tuner("WristPID_D", 0, true);

  protected ArmFeedforward wristFeedforward = new ArmFeedforward(WristFeedforwardkS.get(),
      WristFeedforwardkG.get(), 0);

  protected final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  // protected final DigitalInput homeSwitch = new
  // DigitalInput(Constants.Digital.ALGAE_HOME_LIMIT_SWITCH);
  // protected final AnalogInput distanceSensor = new
  // AnalogInput(Constants.Analog.ALGAE_DISTANCE_SENSOR);

  public GrabberIOReal() {
    SparkMaxConfig grabberConfig = new SparkMaxConfig();

    // NEO 550's have a recommended current limit range of 20-40A.
    grabberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30).voltageCompensation(12.0);

    grabberConfig.inverted(Constants.Grabber.LEFT_GRABBER_INVERTED);
    REVUtils.tryUntilOk(() -> leftGrabberMotor.configure(grabberConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters));

    grabberConfig.follow(Constants.CAN.GRABBER_LEFT_MOTOR, Constants.Grabber.RIGHT_GRABBER_INVERTED);
    REVUtils.tryUntilOk(() -> rightGrabberMotor.configure(grabberConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters));

    SparkMaxConfig wristConfig = new SparkMaxConfig();
    // NEO's have a recommended current limit range of 40-60A, but we deliberately
    // lowered it to 38A for our drive
    // motors. Unclear what this motor limit should be.
    wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    wristConfig.encoder.positionConversionFactor(1 / Constants.Grabber.WRIST_GEAR_RATIO)
        .velocityConversionFactor(1 / Constants.Grabber.WRIST_GEAR_RATIO / 60 * 2 * Math.PI);
    wristConfig.closedLoop.pidf(WristPID_P.get(), 0, WristPID_D.get(), REVUtils.NEO_FF);

    REVUtils.tryUntilOk(
        () -> wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // FIXME: This is just a sanity value, but we should figure out homing.
    wristEncoder.setPosition(0);

    WristFeedforwardkS.addListener((_e) -> updateParams());
    WristFeedforwardkG.addListener((_e) -> updateParams());
    WristPID_P.addListener((_e) -> updateParams());
    WristPID_D.addListener((_e) -> updateParams());
  }

  public void moveTowardsGoal(double goalAngleRadians, double currentAngleRadians) {
    var velocity = Constants.Grabber.WRIST_SPEED_RADIANS_PER_SECOND
        * Math.signum(goalAngleRadians - currentAngleRadians);
    var ff = wristFeedforward.calculate(goalAngleRadians, velocity);
    controller.setReference(goalAngleRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  public void setGrabberMotor(double speed) {
    leftGrabberMotor.set(speed);
  }

  @Override
  public void updateInputs(GrabberIOInputs inputs) {
    inputs.algaeDistance = Sensors.getInstance().getGrabberAcquiredDistance();
    inputs.home = Sensors.getInstance().getGrabberHomeSwitch();
  }

  public void updateParams() {
    var new_config = new SparkMaxConfig();
    new_config.closedLoop.pidf(WristPID_P.get(), 0, WristPID_D.get(), REVUtils.NEO_FF);
    wristMotor.configure(new_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setBrakeMode(boolean mode) {
    wristMotor.configure(new SparkMaxConfig().idleMode(mode ? IdleMode.kBrake : IdleMode.kCoast),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
