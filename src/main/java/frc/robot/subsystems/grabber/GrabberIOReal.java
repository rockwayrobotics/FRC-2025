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
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.REVUtils;
import frc.robot.util.Sensors;
import frc.robot.util.Tuner;

public class GrabberIOReal implements GrabberIO {

  protected final SparkMax leftGrabberMotor = new SparkMax(Constants.CAN.GRABBER_LEFT_MOTOR, MotorType.kBrushless);
  protected final SparkMax rightGrabberMotor = new SparkMax(Constants.CAN.GRABBER_RIGHT_MOTOR, MotorType.kBrushless);
  protected final SparkMax wristMotor = new SparkMax(Constants.CAN.GRABBER_WRIST_MOTOR, MotorType.kBrushless);
  protected final SparkClosedLoopController controller = wristMotor.getClosedLoopController();

  final Tuner wristFeedforwardkS = new Tuner("Grabber/wrist_feedforward_Ks", 0, true);
  final Tuner wristFeedforwardkG = new Tuner("Grabber/wrist_feedforward_Kg", 0, true);
  final Tuner wristPID_P = new Tuner("Grabber/wrist_Kp", 0, true);
  final Tuner wristPID_D = new Tuner("Grabber/wrist_Kd", 0, true);
  final Tuner wristMaxNormalizedSpeed = new Tuner("Grabber/wrist_normalized_speed_max", 0.1, true);
  final Tuner wristMinNormalizedSpeed = new Tuner("Grabber/wrist_normalized_speed_min", -0.1, true);
  final Tuner wristSoftLimitMinAngleRads = new Tuner("Grabber/wrist_soft_limit_min_angle_rads",
      Units.degreesToRadians(-20), true);
  final Tuner wristSoftLimitMaxAngleRads = new Tuner("Grabber/wrist_soft_limit_max_angle_rads",
      Units.degreesToRadians(0), true);

  protected ArmFeedforward wristFeedforward = new ArmFeedforward(wristFeedforwardkS.get(),
      wristFeedforwardkG.get(), 0);

  protected final RelativeEncoder wristEncoder = wristMotor.getEncoder();

  public GrabberIOReal() {
    SparkMaxConfig grabberConfig = new SparkMaxConfig();

    // NEO 550's have a recommended current limit range of 20-40A.
    grabberConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0);

    grabberConfig.inverted(Constants.Grabber.LEFT_GRABBER_INVERTED);
    REVUtils.tryUntilOk(() -> leftGrabberMotor.configure(grabberConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters));

    grabberConfig.follow(Constants.CAN.GRABBER_LEFT_MOTOR, Constants.Grabber.RIGHT_GRABBER_INVERTED);
    REVUtils.tryUntilOk(() -> rightGrabberMotor.configure(grabberConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters));

    updateParams(true);

    // FIXME: This is just a sanity value, but we should figure out homing.
    wristEncoder.setPosition(Units.degreesToRadians(-90));

    wristFeedforwardkS.addListener((_e) -> updateParams(false));
    wristFeedforwardkG.addListener((_e) -> updateParams(false));
    wristPID_P.addListener((_e) -> updateParams(false));
    wristPID_D.addListener((_e) -> updateParams(false));
    wristMaxNormalizedSpeed.addListener((_e) -> updateParams(false));
    wristMinNormalizedSpeed.addListener((_e) -> updateParams(false));
    wristSoftLimitMinAngleRads.addListener((_e) -> updateParams(false));
    wristSoftLimitMaxAngleRads.addListener((_e) -> updateParams(false));
  }

  @Override
  public void moveTowardsGoal(double goalAngleRadians, double currentAngleRadians) {
    var velocity = Constants.Grabber.WRIST_SPEED_RADIANS_PER_SECOND
        * Math.signum(goalAngleRadians - currentAngleRadians);
    var ff = wristFeedforward.calculate(goalAngleRadians, velocity);
    controller.setReference(goalAngleRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  public void stopWrist() {
    wristMotor.set(0);
  }

  @Override
  public void setGrabberMotor(double speed) {
    leftGrabberMotor.set(speed);
  }

  @Override
  public void updateInputs(GrabberIOInputs inputs) {
    REVUtils.ifOk(wristMotor, wristEncoder::getPosition, (value) -> inputs.wristAngleRadians = value);
    REVUtils.ifOk(wristMotor, wristEncoder::getVelocity, (value) -> inputs.wristVelocityRadPerSec = value);
    inputs.algaeDistance = Sensors.getInstance().getGrabberAcquiredDistance();
    inputs.home = Sensors.getInstance().getGrabberHomeSwitch();
  }

  public void updateParams(boolean resetSafe) {
    ResetMode resetMode = resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    wristFeedforward = new ArmFeedforward(wristFeedforwardkS.get(), wristFeedforwardkG.get(), 0);
    SparkMaxConfig wristConfig = new SparkMaxConfig();
    if (resetSafe) {
      // NEO's have a recommended current limit range of 40-60A, but we deliberately
      // lowered it to 38A for our drive
      // motors. Unclear what this motor limit should be.
      wristConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
      wristConfig.encoder.positionConversionFactor(Constants.Grabber.WRIST_CONVERSION_FACTOR)
          .velocityConversionFactor(Constants.Grabber.WRIST_CONVERSION_FACTOR / 60);
      wristConfig.closedLoopRampRate(1);
    }
    // We want position control so no ff term
    wristConfig.closedLoop.pidf(wristPID_P.get(), 0, wristPID_D.get(), 0);
    wristConfig.closedLoop.outputRange(wristMinNormalizedSpeed.get(), wristMaxNormalizedSpeed.get());
    wristConfig.softLimit.forwardSoftLimit(wristSoftLimitMaxAngleRads.get())
        .reverseSoftLimit(wristSoftLimitMinAngleRads.get()).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    REVUtils.tryUntilOk(() -> wristMotor.configure(wristConfig, resetMode, PersistMode.kPersistParameters));
  }

  public void setBrakeMode(boolean mode) {
    wristMotor.configure(new SparkMaxConfig().idleMode(mode ? IdleMode.kBrake : IdleMode.kCoast),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void home() {
    wristEncoder.setPosition(Units.degreesToRadians(-90));
  }
}
