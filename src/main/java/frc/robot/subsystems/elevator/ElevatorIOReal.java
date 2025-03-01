package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;

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
  protected final SparkFlex Motor = new SparkFlex(Constants.CAN.ELEVATOR_MOTOR, MotorType.kBrushless);

  // protected final DigitalInput homeSwitch = new
  // DigitalInput(Constants.Digital.ELEVATOR_HOME_BEAMBREAK);

  final Tuner ElevatorFeedforwardkS = new Tuner("ElevatorFeedforwardkS", 0, true);
  final Tuner ElevatorFeedforwardkG = new Tuner("ElevatorFeedforwardkG", 0, true);
  final Tuner ElevatorPID_P = new Tuner("ElevatorPID_P", 0, true);
  final Tuner ElevatorPID_D = new Tuner("ElevatorPID_D", 0, true);

  protected final RelativeEncoder encoder = Motor.getEncoder();
  protected final SparkClosedLoopController controller = Motor.getClosedLoopController();
  // TODO: tune
  protected ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorFeedforwardkS.get(),
      ElevatorFeedforwardkG.get(), 0);
  protected final double SPEED_METERS_PER_SECOND = 0.5;

  public ElevatorIOReal() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    config.encoder.positionConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO)
        .velocityConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO / 60);

    config.closedLoop.pidf(ElevatorPID_P.get(), 0, ElevatorPID_D.get(), REVUtils.VORTEX_FF);

    config.encoder.positionConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO)
        .velocityConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO / 60);
    REVUtils.tryUntilOk(
        () -> Motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    REVUtils.tryUntilOk(() -> encoder.setPosition(0.0));

    ElevatorFeedforwardkS.addListener((_e) -> updateParams());
    ElevatorFeedforwardkG.addListener((_e) -> updateParams());
    ElevatorPID_P.addListener((_e) -> updateParams());
    ElevatorPID_D.addListener((_e) -> updateParams());
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.homed = Sensors.getInstance().getElevatorHomeBeambreak();
    // FIXME: Measure CAN bus usage with all these queries?
    REVUtils.ifOk(Motor, encoder::getPosition, (value) -> inputs.positionMeters = value);
    REVUtils.ifOk(Motor, encoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    REVUtils.ifOk(Motor, new DoubleSupplier[] {
        Motor::getAppliedOutput, Motor::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(Motor, Motor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
    // FIXME: Could ask for temperature?
    // REVUtils.ifOk(motor, motor::getMotorTemperature, (value) ->
    // inputs.tempCelsius = value);
  }

  @Override
  public void moveTowardsGoal(double goalHeightMeters, double currentHeightMeters) {
    var velocity = SPEED_METERS_PER_SECOND * Math.signum(goalHeightMeters - currentHeightMeters);
    var ff = feedforward.calculate(velocity);
    controller.setReference(goalHeightMeters, ControlType.kPosition, ClosedLoopSlot.kSlot0, ff);
  }

  public void setVoltage(double volts) {
    controller.setReference(volts, ControlType.kVoltage);
  }

  @Override
  public void stop() {
    Motor.set(0);
  }

  public void updateParams() {
    feedforward = new ElevatorFeedforward(ElevatorFeedforwardkS.get(), ElevatorFeedforwardkG.get(), 0);
    var new_config = new SparkMaxConfig();
    new_config.closedLoop.pidf(ElevatorPID_P.get(), 0, ElevatorPID_D.get(), REVUtils.VORTEX_FF);
    Motor.configure(new_config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
}
