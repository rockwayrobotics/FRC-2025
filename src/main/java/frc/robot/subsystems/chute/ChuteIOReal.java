package frc.robot.subsystems.chute;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.DigitalInput;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;
import frc.robot.util.REVUtils;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ChuteIOReal implements ChuteIO {
  protected final DigitalInput coralShootSensor = new DigitalInput(Constants.Digital.CORAL_SHOOT_SENSOR);
  protected final SparkMax pivotMotor = new SparkMax(Constants.CAN.CORAL_PIVOT_MOTOR, MotorType.kBrushless);
  protected final SparkMax shooterMotor = new SparkMax(Constants.CAN.CORAL_SHOOT_MOTOR, MotorType.kBrushless);

  protected final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();
  protected final RelativeEncoder shooterEncoder = shooterMotor.getEncoder();

  protected final SparkClosedLoopController pivotController = pivotMotor.getClosedLoopController();

  public ChuteIOReal() {
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    pivotConfig.encoder.positionConversionFactor(1 / Constants.Coral.PIVOT_GEAR_RATIO);
    pivotConfig.closedLoop.pidf(0.5, 0, 0, REVUtils.SPARK_MAX_FF);
    REVUtils.tryUntilOk(() -> pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));

    SparkMaxConfig shooterConfig = new SparkMaxConfig();
    shooterConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(38).voltageCompensation(12.0);
    REVUtils.tryUntilOk(() -> shooterMotor.configure(shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  @Override
  public void updateInputs(CoralIOInputs inputs) {
    REVUtils.ifOk(pivotMotor, pivotEncoder::getPosition, (value) -> inputs.pivotAngleRadians = value);
    REVUtils.ifOk(pivotMotor, pivotEncoder::getVelocity, (value) -> inputs.pivotVelocityRadPerSec = value / 60.0);
    REVUtils.ifOk(shooterMotor, shooterEncoder::getVelocity, (value) -> inputs.shooterVelocityRadPerSec = value);
    // FIXME: Should we be reading this at 50Hz?
    inputs.coralLoaded = coralShootSensor.get();
  }

  @Override
  public void setPivotGoal(double angleRadians) {
    pivotController.setReference(angleRadians, ControlType.kPosition);
  }

  @Override
  public void setShooterVoltage(double voltage) {
    shooterMotor.setVoltage(voltage);
  }
}
