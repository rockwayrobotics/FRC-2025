package frc.robot.subsystems.drive;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Constants;
import frc.robot.util.REVUtils;

public class DriveIOSparkMax implements DriveIO {
  protected final DifferentialDrive differentialDrive;

  protected final SparkMax leftDriveMotorF = new SparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_F, MotorType.kBrushless);
  protected final SparkMax leftDriveMotorR = new SparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_R, MotorType.kBrushless);
  protected final SparkMax rightDriveMotorF = new SparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_F, MotorType.kBrushless);
  protected final SparkMax rightDriveMotorR = new SparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_R, MotorType.kBrushless);

  protected final RelativeEncoder leftEncoder = leftDriveMotorF.getEncoder();
  protected final RelativeEncoder rightEncoder = rightDriveMotorF.getEncoder();

  protected final SparkClosedLoopController leftController = leftDriveMotorF.getClosedLoopController();
  protected final SparkClosedLoopController rightController = rightDriveMotorF.getClosedLoopController();

  protected final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(Constants.Drive.REAL_KS, Constants.Drive.REAL_KV);

  public DriveIOSparkMax() {
    var config = new SparkMaxConfig();
    // FIXME: Move to constants
    // FIXME: Do we want to be able to switch out of brake mode?
    config.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    // FIXME: Measure this and consider using it?
    // FIXME: make FF a constant?
    config.closedLoop.pidf(0.5, 0, 0, 1 / 473);
    config.encoder
        // Encoder rotations in radians converted to meters
        .positionConversionFactor(Constants.Drive.WHEEL_CIRCUM_CM / 100 / Constants.Drive.WHEEL_GEAR_RATIO)
        // Encoder velocity in revolutions per minute converted to m/s
        .velocityConversionFactor(Constants.Drive.WHEEL_CIRCUM_CM / 100 / Constants.Drive.WHEEL_GEAR_RATIO / 60);
    // The template suggests these changes to measurement sampling rate:
    // .uvwMeasurementPeriod(10) // defaults to 32ms
    // .uvwAverageDepth(2); // defaults to 3
    // UVW average depth appears to be a power of 2 - so "2" actually yields 4
    // measurements per period, and
    // 3 yields 8 measurements per period. The velocity is averaged over the period
    // based on the measurements.
    // This seems like it would provide better responsiveness to changes, but may
    // increase the noise in
    // measurements at low speeds.

    config.inverted(Constants.Drive.LEFT_DRIVE_INVERTED);
    REVUtils.tryUntilOk(
        () -> leftDriveMotorF.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.inverted(Constants.Drive.RIGHT_DRIVE_INVERTED);
    REVUtils.tryUntilOk(
        () -> rightDriveMotorF.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.inverted(Constants.Drive.LEFT_DRIVE_INVERTED).follow(leftDriveMotorF);
    REVUtils.tryUntilOk(
        () -> leftDriveMotorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
    config.inverted(Constants.Drive.RIGHT_DRIVE_INVERTED).follow(rightDriveMotorF);
    REVUtils.tryUntilOk(
        () -> rightDriveMotorR.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    // At creation time, set encoder positions to 0
    REVUtils.tryUntilOk(() -> leftEncoder.setPosition(0.0));
    REVUtils.tryUntilOk(() -> rightEncoder.setPosition(0.0));

    differentialDrive = new DifferentialDrive(leftDriveMotorF, rightDriveMotorF);

  }

  @Override
  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    REVUtils.ifOk(leftDriveMotorF, leftEncoder::getPosition, (value) -> inputs.leftPositionMeters = value);
    REVUtils.ifOk(leftDriveMotorF, leftEncoder::getVelocity, (value) -> inputs.leftVelocityMetersPerSec = value);
    REVUtils.ifOk(leftDriveMotorF, new DoubleSupplier[] {
        leftDriveMotorF::getAppliedOutput, leftDriveMotorF::getBusVoltage
    }, (values) -> inputs.leftAppliedVolts = values[0] * values[1]);
    REVUtils.ifOk(leftDriveMotorF, new DoubleSupplier[] {
        leftDriveMotorF::getOutputCurrent, leftDriveMotorR::getOutputCurrent
    }, (values) -> inputs.leftCurrentAmps = values);

    REVUtils.ifOk(rightDriveMotorF, rightEncoder::getPosition, (value) -> inputs.rightPositionMeters = value);
    REVUtils.ifOk(rightDriveMotorF, rightEncoder::getVelocity, (value) -> inputs.rightVelocityMetersPerSec = value);
    REVUtils.ifOk(rightDriveMotorF, new DoubleSupplier[] {
        rightDriveMotorF::getAppliedOutput, rightDriveMotorF::getBusVoltage
    }, (values) -> inputs.rightAppliedVolts = values[0] * values[1]);
    REVUtils.ifOk(rightDriveMotorF, new DoubleSupplier[] {
        rightDriveMotorF::getOutputCurrent, rightDriveMotorR::getOutputCurrent
    }, (values) -> inputs.rightCurrentAmps = values);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftDriveMotorF.setVoltage(leftVolts);
    rightDriveMotorF.setVoltage(rightVolts);
    differentialDrive.feed();
  }

  @Override
  public void setVelocity(double leftMetersPerSec, double rightMetersPerSec, double leftFFVolts, double rightFFVolts) {
    leftController.setReference(leftMetersPerSec, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, leftFFVolts);
    rightController.setReference(rightMetersPerSec, ControlType.kVelocity,
        ClosedLoopSlot.kSlot0, rightFFVolts);

    // leftController.setReference(leftMetersPerSec, ControlType.kVelocity);
    // rightController.setReference(rightMetersPerSec, ControlType.kVelocity);

    // leftDriveMotorF.setVoltage(leftFFVolts);
    // rightDriveMotorF.setVoltage(rightFFVolts);
    differentialDrive.feed();
  }

  public SimpleMotorFeedforward getFeedForward() {
    return feedForward;
  }
}
