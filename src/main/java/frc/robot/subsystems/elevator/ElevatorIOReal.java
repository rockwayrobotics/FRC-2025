package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.util.REVUtils;
import frc.robot.util.Sensors;
import frc.robot.util.Tuner;

public class ElevatorIOReal implements ElevatorIO {
  // Note that we may eventually have a second motor on the elevator
  protected final SparkFlex motor = new SparkFlex(Constants.CAN.ELEVATOR_MOTOR, MotorType.kBrushless);

  final Tuner elevatorFeedforwardkS = new Tuner("Elevator/feedforward_kS", 0, true);
  final Tuner elevatorFeedforwardkG = new Tuner("Elevator/feedforward_kG", 0, true);
  final Tuner elevatorPID_P = new Tuner("Elevator/Kp", 0, true);
  final Tuner elevatorPID_D = new Tuner("Elevator/Kd", 0, true);
  final Tuner elevatorMaxNormalizedSpeed = new Tuner("Elevator/normalized_speed_max", 0.1, true);
  final Tuner elevatorMinNormalizedSpeed = new Tuner("Elevator/normalized_speed_min", -0.1, true);
  final Tuner elevatorSoftLimitMin = new Tuner("Elevator/soft_limit_min_mm", 5, true);
  final Tuner elevatorSoftLimitMax = new Tuner("Elevator/soft_limit_max_mm", 1300, true);

  protected final RelativeEncoder encoder = motor.getEncoder();
  protected final SparkClosedLoopController controller = motor.getClosedLoopController();
  protected ElevatorFeedforward feedforward;

  // FIXME: Not used and not measured - only needs to be arbitrary positive
  protected final double SPEED_MM_PER_SEC = 0.5;

  public ElevatorIOReal() {
    updateParams(true);

    REVUtils.tryUntilOk(() -> encoder.setPosition(0.0));

    elevatorFeedforwardkS.addListener((_e) -> updateParams(false));
    elevatorFeedforwardkG.addListener((_e) -> updateParams(false));
    elevatorPID_P.addListener((_e) -> updateParams(false));
    elevatorPID_D.addListener((_e) -> updateParams(false));
    elevatorMaxNormalizedSpeed.addListener((_e) -> updateParams(false));
    elevatorMinNormalizedSpeed.addListener((_e) -> updateParams(false));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.homed = Sensors.getInstance().getElevatorHomeBeambreak();
    if (inputs.homed) {
      // REVUtils.tryUntilOk(() -> encoder.setPosition(0.0));
    }

    // FIXME: Measure CAN bus usage with all these queries?
    REVUtils.ifOk(motor, encoder::getPosition, (value) -> inputs.positionMillimeters = value);
    REVUtils.ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityMillimetersPerSec = value);
    REVUtils.ifOk(motor, new DoubleSupplier[] {
        motor::getAppliedOutput, motor::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
    // FIXME: Could ask for temperature?
    // REVUtils.ifOk(motor, motor::getMotorTemperature, (value) ->
    // inputs.tempCelsius = value);
  }

  @Override
  public void moveTowardsGoal(double goalHeightMillimeters, double currentHeightMillimeters) {
    var velocity = SPEED_MM_PER_SEC * Math.signum(goalHeightMillimeters - currentHeightMillimeters);
    var ff = feedforward.calculate(velocity);
    controller.setReference(goalHeightMillimeters, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  public void setVoltage(double volts) {
    controller.setReference(volts, ControlType.kVoltage);
  }

  @Override
  public void stop() {
    motor.set(0);
  }

  public void updateParams(boolean resetSafe) {
    ResetMode resetMode = resetSafe ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    feedforward = new ElevatorFeedforward(elevatorFeedforwardkS.get(), elevatorFeedforwardkG.get(), 0);
    SparkMaxConfig config = new SparkMaxConfig();
    if (resetSafe) {
      config.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0).inverted(true);

      config.encoder.positionConversionFactor(Constants.Elevator.ELEVATOR_CONVERSION_FACTOR).velocityConversionFactor(
          Constants.Elevator.ELEVATOR_CONVERSION_FACTOR / 60);
    }

    config.softLimit.forwardSoftLimit(elevatorSoftLimitMax.get()).reverseSoftLimit(elevatorSoftLimitMin.get())
        .forwardSoftLimitEnabled(true).reverseSoftLimitEnabled(true);

    // No ff term here because we want position control not velocity
    config.closedLoop.pidf(elevatorPID_P.get(), 0, elevatorPID_D.get(), 0);
    config.closedLoop.outputRange(elevatorMinNormalizedSpeed.get(), elevatorMaxNormalizedSpeed.get());
    REVUtils.tryUntilOk(() -> motor.configure(config, resetMode, PersistMode.kPersistParameters));
  }
}
