package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.util.REVUtils;
import frc.robot.util.Tuner;

public class ElevatorIOReal implements ElevatorIO {
  // Note that we may eventually have a second motor on the elevator
  protected final SparkMax motorMain = new SparkMax(Constants.CAN.ELEVATOR_MOTOR_FRONT, MotorType.kBrushless);
  protected final SparkMax motorFollower = new SparkMax(Constants.CAN.ELEVATOR_MOTOR_BACK, MotorType.kBrushless);

  final Tuner elevatorFeedforwardkS = new Tuner("Elevator/feedforward_Ks", 0.1, true);
  final Tuner elevatorFeedforwardkG = new Tuner("Elevator/feedforward_Kg", 0.4, true);
  final Tuner elevatorFeedforwardkV = new Tuner("Elevator/feedforward_Kv", 0.01427, true);
  final Tuner elevatorPID_P = new Tuner("Elevator/Kp", 0.00136, true);
  final Tuner elevatorPID_I = new Tuner("Elevator/Ki", 0, true);
  final Tuner elevatorPID_D = new Tuner("Elevator/Kd", 0.005, true);
  final Tuner elevatorMaxSpeedMmPerSecond = new Tuner("Elevator/speed_max_mm_per_second", 100, true);
  final Tuner elevatorMaxAccelerationMmPerSecond2 = new Tuner("Elevator/accel_max_mm_per_second2", 100, true);
  final Tuner elevatorSoftLimitMin = new Tuner("Elevator/soft_limit_min_mm", 20, true);
  final Tuner elevatorSoftLimitMax = new Tuner("Elevator/soft_limit_max_mm", 1200, true);

  protected final RelativeEncoder encoder = motorMain.getEncoder();
  protected final SparkClosedLoopController controller = motorMain.getClosedLoopController();
  protected ElevatorFeedforward feedforward;

  // FIXME: Not used and not measured - only needs to be arbitrary positive
  protected final double SPEED_MM_PER_SEC = 0.5;

  public ElevatorIOReal() {
    updateParams(true);

    REVUtils.tryUntilOk(() -> encoder.setPosition(0.0));
    var followConfig = new SparkMaxConfig().follow(motorMain);
    REVUtils.tryUntilOk(
        () -> motorFollower.configure(followConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    elevatorFeedforwardkS.addListener((_e) -> updateParams(false));
    elevatorFeedforwardkG.addListener((_e) -> updateParams(false));
    elevatorFeedforwardkV.addListener((_e) -> updateParams(false));
    elevatorPID_P.addListener((_e) -> updateParams(false));
    elevatorPID_I.addListener((_e) -> updateParams(false));
    elevatorPID_D.addListener((_e) -> updateParams(false));
    elevatorMaxSpeedMmPerSecond.addListener((_e) -> updateParams(false));
    elevatorMaxAccelerationMmPerSecond2.addListener((_e) -> updateParams(false));
    elevatorSoftLimitMin.addListener((_e) -> updateParams(false));
    elevatorSoftLimitMax.addListener((_e) -> updateParams(false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // inputs.homeBeamBroken = Sensors.getInstance().getElevatorHomeBeambroken();
    // if (inputs.homeBeamBroken) {
    // // REVUtils.tryUntilOk(() -> encoder.setPosition(0.0));
    // }

    // FIXME: Measure CAN bus usage with all these queries?
    REVUtils.ifOk(motorMain, encoder::getPosition, (value) -> inputs.positionMillimeters = value);
    REVUtils.ifOk(motorMain, encoder::getVelocity, (value) -> inputs.velocityMillimetersPerSec = value);
    REVUtils.ifOk(motorMain, new DoubleSupplier[] {
        motorMain::getAppliedOutput, motorMain::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(motorMain, motorMain::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
    // FIXME: Could ask for temperature?
    // REVUtils.ifOk(motor, motor::getMotorTemperature, (value) ->
    // inputs.tempCelsius = value);
  }

  @Override
  public void moveTowardsGoal(double goalHeightMillimeters, double currentHeightMillimeters) {
    var velocity = SPEED_MM_PER_SEC * Math.signum(goalHeightMillimeters - currentHeightMillimeters);
    var ff = feedforward.calculate(velocity);
    controller.setReference(goalHeightMillimeters, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0, ff);
  }

  public void setVoltage(double volts) {
    controller.setReference(volts, ControlType.kVoltage);
  }

  @Override
  public void stop() {
    motorMain.set(0);
  }

  @Override
  public void setMaxSpeedTuner(double speedMmPerSecond) {
    elevatorMaxSpeedMmPerSecond.set(speedMmPerSecond);
  }

  @Override
  public double getMaxSpeedTuner() {
    return elevatorMaxSpeedMmPerSecond.get();
  }

  public void updateParams(boolean resetSafe) {
    ResetMode resetMode = resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    feedforward = new ElevatorFeedforward(elevatorFeedforwardkS.get(), elevatorFeedforwardkG.get(),
        elevatorFeedforwardkV.get());
    SparkMaxConfig config = new SparkMaxConfig();
    if (resetSafe) {
      config.idleMode(IdleMode.kBrake).smartCurrentLimit(60).voltageCompensation(12.0).inverted(true);
      // FIXME: apply config to both motors

      config.encoder.positionConversionFactor(Constants.Elevator.ELEVATOR_CONVERSION_FACTOR).velocityConversionFactor(
          Constants.Elevator.ELEVATOR_CONVERSION_FACTOR / 60);
    }

    config.softLimit.forwardSoftLimit(elevatorSoftLimitMax.get()).reverseSoftLimit(elevatorSoftLimitMin.get())
        .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    // No ff term here because we want position control not velocity
    config.closedLoop.pidf(elevatorPID_P.get(), elevatorPID_I.get(), elevatorPID_D.get(), 0).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    config.closedLoop.maxMotion
      .maxVelocity(elevatorMaxSpeedMmPerSecond.get())
      .maxAcceleration(elevatorMaxAccelerationMmPerSecond2.get())
      .allowedClosedLoopError(10);
    REVUtils.tryUntilOk(() -> motorMain.configure(config, resetMode, PersistMode.kPersistParameters));
  }

  public void zeroEncoder() {
    encoder.setPosition(0);
  }
}
