package frc.robot.subsystems.chute;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.Future;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;

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
  final Tuner pivotSoftLimitMinAngleRads = new Tuner("Chute/pivot_soft_limit_min_angle_rads",
      Units.degreesToRadians(-10),
      true);
  final Tuner pivotSoftLimitMaxAngleRads = new Tuner("Chute/pivot_soft_limit_max_angle_rads",
      Units.degreesToRadians(10),
      true);

  protected ArmFeedforward pivotFeedforward;

  public ChuteIOReal() {
    updateParams(true);

    // At creation time, set encoder positions to our initial position
    REVUtils.tryUntilOk(() -> pivotEncoder.setPosition(Constants.Chute.PIVOT_INITIAL_ANGLE_RADS));

    pivotFeedforwardkS.addListener((_e) -> updateParams(false));
    pivotFeedforwardkG.addListener((_e) -> updateParams(false));
    pivotPID_P.addListener((_e) -> updateParams(false));
    pivotPID_D.addListener((_e) -> updateParams(false));
    pivotMaxNormalizedSpeed.addListener((_e) -> updateParams(false));
    pivotMinNormalizedSpeed.addListener((_e) -> updateParams(false));
    pivotSoftLimitMinAngleRads.addListener((_e) -> updateParams(false));
    pivotSoftLimitMaxAngleRads.addListener((_e) -> updateParams(false));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    inputs.homeSwitchPressed = Sensors.getInstance().getChuteHomeSwitch();
    if (inputs.homeSwitchPressed) {
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
      pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0).inverted(true);
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

  public CompletableFuture<Boolean> home() {
    var promise = new CompletableFuture<Boolean>();
    pivotEncoder.setPosition(0);
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.softLimit.forwardSoftLimit(Units.degreesToRadians(10))
        .reverseSoftLimit(Units.degreesToRadians(-10)).forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);
    pivotConfig.smartCurrentLimit(10);

    REVUtils.tryUntilOk(
        () -> pivotMotor.configure(pivotConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));

    if (Sensors.getInstance().getChuteHomeSwitch()) { // home switch is pressed
      pivotMotor.set(0.05); // away from the home switch
      Sensors.getInstance().registerChuteHomeInterrupt((interrupt, rising, falling) -> {
        if (falling) {
          System.out.println("home switch is pressed, falling. start");
          pivotMotor.set(0);
          pivotEncoder.setPosition(0);

          updateParams(true);

          promise.complete(true);
          interrupt.close();
          System.out.println("home switch is pressed, falling. done");
        } else {
          System.err.println("something bad happened, home switch was pressed, rising edge");
          promise.complete(false);
        }
      });
    } else if (!Sensors.getInstance().getChuteHomeSwitch()) {
      pivotMotor.set(-0.05); // to the home switch
      Sensors.getInstance().registerChuteHomeInterrupt((interrupt, rising, falling) -> {
        if (falling) {
          System.out.println("home switch started not pressed, it is now unpressed, falling. start");
          pivotMotor.set(0);
          pivotEncoder.setPosition(0);

          updateParams(true);

          promise.complete(true);
          interrupt.close();
          System.out.println("home switch started not pressed, it is now unpressed, falling. done");
        } else if (rising) {
          System.out.println("home switch started not pressed, it is now pressed , rising. done");
          pivotMotor.set(0.05); // away from the home switch
        }
      });
    }
    return promise;
  }
}
