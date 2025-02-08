package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;


import frc.robot.Constants;
import frc.robot.util.REVUtils;

public class ElevatorIOSparkMax implements ElevatorIO {
  protected final SparkMax motor = new SparkMax(Constants.CAN.ELEVATOR_MOTOR, MotorType.kBrushless);

  protected final RelativeEncoder encoder = motor.getEncoder();

  protected final SparkClosedLoopController controller = motor.getClosedLoopController();

  public ElevatorIOSparkMax() {
    var config = new SparkMaxConfig();
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    config.encoder.positionConversionFactor(Constants.Elevator.DRUM_RADIUS_METERS / Constants.Elevator.GEAR_RATIO)
        .velocityConversionFactor(Constants.Elevator.DRUM_RADIUS_METERS / Constants.Elevator.GEAR_RATIO / 60);
    config.closedLoop.pidf(0.5, 0, 0, REVUtils.SPARK_MAX_FF);
    REVUtils.tryUntilOk(() -> motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    // FIXME: Measure CAN bus usage with all these queries?
    REVUtils.ifOk(motor, encoder::getPosition, (value) -> inputs.positionMeters = value);
    REVUtils.ifOk(motor, encoder::getVelocity, (value) -> inputs.velocityMetersPerSec = value);
    REVUtils.ifOk(motor, new DoubleSupplier[] {
        motor::getAppliedOutput, motor::getBusVoltage
    }, (values) -> inputs.appliedVoltage = values[0] * values[1]);
    REVUtils.ifOk(motor, motor::getOutputCurrent, (value) -> inputs.supplyCurrentAmps = value);
    // FIXME: Could ask for temperature?
    // REVUtils.ifOk(motor, motor::getMotorTemperature, (value) -> inputs.tempCelsius = value);
  }

  @Override
  public void setGoal(double positionMeters) {
    controller.setReference(positionMeters, ControlType.kPosition);
  }

  @Override
  public void stop() {
    motor.set(0);
  }
}
