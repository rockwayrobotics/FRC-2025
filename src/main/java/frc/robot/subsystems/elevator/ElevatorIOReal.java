package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import java.util.function.DoubleSupplier;

import frc.robot.Constants;
import frc.robot.util.REVUtils;

public class ElevatorIOReal implements ElevatorIO {
  protected final SparkFlex leftMotor = new SparkFlex(Constants.CAN.ELEVATOR_MOTOR_LEFT, MotorType.kBrushless);
  // FIXME: We don't have a second motor on the elevator yet, but we may yet have one
  // protected final SparkFlex rightMotor = new SparkFlex(Constants.CAN.ELEVATOR_MOTOR_RIGHT, MotorType.kBrushless);

  protected final RelativeEncoder encoder = leftMotor.getEncoder();

  protected final SparkClosedLoopController controller = leftMotor.getClosedLoopController();

  public ElevatorIOReal() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    config.encoder.positionConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO)
        .velocityConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO / 60);
    config.closedLoop.pidf(0.5, 0, 0, REVUtils.VORTEX_FF);
    config.encoder.positionConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO)
        .velocityConversionFactor(Constants.Elevator.SPROCKET_RADIUS_METERS / Constants.Elevator.GEAR_RATIO / 60);
    REVUtils.tryUntilOk(
        () -> leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    

    REVUtils.tryUntilOk(() -> encoder.setPosition(0.0));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // FIXME: Measure CAN bus usage with all these queries?
    REVUtils.ifOk(leftMotor, encoder::getPosition, (value) -> inputs.positionMeters = value);
    REVUtils.ifOk(leftMotor, encoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    REVUtils.ifOk(leftMotor, new DoubleSupplier[] {
        leftMotor::getAppliedOutput, leftMotor::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(leftMotor, leftMotor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
    // FIXME: Could ask for temperature?
    // REVUtils.ifOk(motor, motor::getMotorTemperature, (value) ->
    // inputs.tempCelsius = value);
  }

  @Override
  public void setGoal(double positionMeters) {
    controller.setReference(positionMeters, ControlType.kPosition);
  }

  @Override
  public void stop() {
    leftMotor.set(0);
  }
}
