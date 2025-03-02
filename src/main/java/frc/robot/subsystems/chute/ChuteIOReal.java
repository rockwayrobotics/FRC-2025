package frc.robot.subsystems.chute;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.util.REVUtils;
import frc.robot.util.Sensors;
import frc.robot.util.Tuner;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ChuteIOReal implements ChuteIO {
  protected final SparkMax pivotMotor = new SparkMax(Constants.CAN.PIVOT_MOTOR, MotorType.kBrushless);
  protected final SparkMax shooterMotor = new SparkMax(Constants.CAN.SHOOTER_MOTOR, MotorType.kBrushless);

  protected final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  protected final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

  protected final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  final Tuner pivotFeedforwardkS = new Tuner("Chute/pivot_feedforward_Ks", 0, true);
  final Tuner pivotFeedforwardkG = new Tuner("Chute/pivot_feedforward_Kg", 0, true);
  final Tuner pivotPID_P = new Tuner("Chute/pivot_Kp", 0, true);
  final Tuner pivotPID_D = new Tuner("Chute/pivot_Kd", 0, true);
  final Tuner pivotMaxNormalizedSpeed = new Tuner("Chute/pivot_normalized_speed_max", 0.1, true);
  final Tuner pivotMinNormalizedSpeed = new Tuner("Chute/pivot_normalized_speed_min", -0.1, true);
  final Tuner pivotSoftLimitMinAngleRads = new Tuner("Chute/soft_limit_min_angle_rads",
      Radians.convertFrom(-10, Degrees),
      true);
  final Tuner pivotSoftLimitMaxAngleRads = new Tuner("Chute/soft_limit_max_angle_rads",
      Radians.convertFrom(10, Degrees),
      true);

  protected ArmFeedforward pivotFeedforward;

  public ChuteIOReal() {
    updateParams(true);

    // At creation time, set encoder positions to our initial position
    REVUtils.tryUntilOk(() -> pivotEncoder.setPosition(Constants.Chute.PIVOT_INITIAL_ANGLE_RADS));

    pivotPID_P.addListener((_e) -> updateParams(false));
    pivotPID_D.addListener((_e) -> updateParams(false));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.homed = Sensors.getInstance().getChuteHomeSwitch();
    if (inputs.homed) {
      // REVUtils.tryUntilOk(() ->
      // pivotEncoder.setPosition(Constants.Chute.PIVOT_INITIAL_ANGLE_RADS));
    }

    REVUtils.ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotAngleRadians = value);
    REVUtils.ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotVelocityRadPerSec = value);
    REVUtils.ifOk(shooterMotor, shooterEncoder::getVelocity, (value) -> inputs.shooterVelocityRadPerSec = value);
    // FIXME: Should we be reading this at 50Hz?
    inputs.coralLoading = Sensors.getInstance().getChuteCoralLoadedBeambreak();
    inputs.coralReady = Sensors.getInstance().getChuteCoralReadyBeambreak();
  }

  @Override
  public void moveTowardsPivotGoal(double goalAngleRadians, double currentAngleRadians) {
    // Arm feed forward expects 0 to be parallel to the floor, but for us, 0 is
    // pointed straight down.
    var ff = pivotFeedforward.calculate(currentAngleRadians - Math.PI / 2,
        Math.signum(goalAngleRadians - currentAngleRadians));
    pivotController.setReference(goalAngleRadians, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  @Override
  public void stopPivot() {
    pivotMotor.set(0);
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterMotor.set(speed);
  }

  private void updateParams(boolean resetSafe) {
    ResetMode resetMode = resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    pivotFeedforward = new ArmFeedforward(pivotFeedforwardkS.get(), pivotFeedforwardkG.get(), 0);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    if (resetSafe) {
      pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
      pivotConfig.encoder.positionConversionFactor(2 * Math.PI / Constants.Chute.PIVOT_GEAR_RATIO)
          .velocityConversionFactor(2 * Math.PI / Constants.Chute.PIVOT_GEAR_RATIO / 60);
    }
    // No ff term here because we want position control not velocity
    pivotConfig.closedLoop.pidf(pivotPID_P.get(), 0, pivotPID_D.get(), 0);
    pivotConfig.closedLoop.outputRange(pivotMinNormalizedSpeed.get(), pivotMaxNormalizedSpeed.get());
    pivotConfig.softLimit.forwardSoftLimit(pivotSoftLimitMaxAngleRads.get())
        .reverseSoftLimit(pivotSoftLimitMinAngleRads.get()).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    REVUtils.tryUntilOk(
        () -> pivotMotor.configure(pivotConfig, resetMode, PersistMode.kPersistParameters));

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    REVUtils.tryUntilOk(
        () -> shooterMotor.configure(shooterConfig, resetMode, PersistMode.kPersistParameters));
  }

  public void setBrakeMode(boolean mode) {
    pivotMotor.configure(new SparkMaxConfig().idleMode(mode ? IdleMode.kBrake : IdleMode.kCoast),
        ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}
