package frc.robot.subsystems;

import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Shooter.ScoringMode;

public class AnglerPIDSubsystem extends PIDSubsystem {

  private final SparkMax m_angleMotor = new SparkMax(Constants.CAN.GEAR, MotorType.kBrushless);
  RelativeEncoder m_angleEncoder = m_angleMotor.getEncoder();

  public GenericEntry speakerAngleWidget;
  public GenericEntry ampAngleWidget;
  public GenericEntry angleEncoderWidget;
  public GenericEntry angleSetpointWidget;
  public GenericEntry angleResetWidget;
  public GenericEntry outputWidget;
  public GenericEntry outputClampedWidget;
  public GenericEntry errorWidget;
  public GenericEntry setpoint1Widget;
  public GenericEntry setpoint2Widget;

  public GenericEntry kPWidget;
  public GenericEntry kIWidget;
  public GenericEntry kDWidget;

  public GenericEntry positiveClampWidget;
  public GenericEntry negativeClampWidget;

  public GenericEntry softLimitOverride;

  private boolean isAngleReset;
  private double kPVal;
  private double kIVal;
  private double kDVal;

  private double setpoint1;
  private double setpoint2;

  private double positiveClamp;
  private double negativeClamp;

  // public double speakerAngleSetpoint;
  // public double ampAngleSetpoint;
  public double angleSetpoint;
  public ScoringMode m_ScoringMode = ScoringMode.SPEAKER;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("Angler");

  public AnglerPIDSubsystem() {
    super(new PIDController(Constants.Angler.DEFAULT_PID_kP, Constants.Angler.DEFAULT_PID_kI,
        Constants.Angler.DEFAULT_PID_kD));
    // getController().setTolerance(Constants.Angler.ANGLE_PID_TOLERANCE);

    SoftLimitConfig softLimit = new SoftLimitConfig()
      .forwardSoftLimit(Constants.Angler.ANGLE_BOTTOM_MAX)
      .reverseSoftLimit(Constants.Angler.ANGLE_TOP_MAX)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);
    m_angleMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake).apply(softLimit), ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    m_angleEncoder.setPosition(0);

    speakerAngleWidget = dashboardTab.addPersistent("Speaker angle", 0).withPosition(0, 0).getEntry();
    ampAngleWidget = dashboardTab.addPersistent("Amp angle", 0).withPosition(0, 0).getEntry();
    angleEncoderWidget = dashboardTab.addPersistent("Angle Encoder", 0).getEntry();
    angleResetWidget = dashboardTab.add("Angle Reset", false).getEntry();

    setpoint1Widget = dashboardTab.addPersistent("Setpoint 1", 0).getEntry();
    setpoint2Widget = dashboardTab.addPersistent("Setpoint 2", 0).getEntry();
    angleSetpointWidget = dashboardTab.add("Angle Setpoint", 0).getEntry();

    outputWidget = dashboardTab.add("Output value", 0).getEntry();
    outputClampedWidget = dashboardTab.add("Output Value Clamped", 0).getEntry();
    errorWidget = dashboardTab.add("PID Error Pos", 0).getEntry();

    kPWidget = dashboardTab.addPersistent("kP Value", Constants.Angler.DEFAULT_PID_kP).getEntry();
    kIWidget = dashboardTab.addPersistent("kI Value", Constants.Angler.DEFAULT_PID_kI).getEntry();
    kDWidget = dashboardTab.addPersistent("kD Value", Constants.Angler.DEFAULT_PID_kD).getEntry();

    positiveClampWidget = dashboardTab.addPersistent("Positive Clamp", Constants.Angler.DEFAULT_POSITIVE_CLAMP)
        .getEntry();
    negativeClampWidget = dashboardTab.addPersistent("Negative Clamp", Constants.Angler.DEFAULT_NEGATIVE_CLAMP)
        .getEntry();

    SmartDashboard.putData("Put Back To Zero", new InstantCommand(() -> angleSetpointWidget.setDouble(0)));
    SmartDashboard.putData("Setpoint 1", new InstantCommand(() -> angleSetpointWidget.setDouble(setpoint1)));
    SmartDashboard.putData("Setpoint 2", new InstantCommand(() -> angleSetpointWidget.setDouble(setpoint2)));

    softLimitOverride = dashboardTab.add("Angler Soft Limit Override", false)
        .getEntry();

    enable();
  }

  public void setScoringMode(ScoringMode mode) {
    m_ScoringMode = mode;
  }

  public void setAngleMotor(double speed) {
    // if (bottomShooterLimitPressed && Math.abs(speed) < 0){
    // m_angleMotor.set(0);
    // } else {

    // System.out.println("Angle: " + speed);

    m_angleMotor.set(speed);
    // }
  }

  public double getAngleEncoder() {
    return m_angleEncoder.getPosition();
  }

  public void resetAngleEncoder() {
    m_angleEncoder.setPosition(0);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    outputWidget.setDouble(output);
    output = MathUtil.clamp(output, negativeClamp, positiveClamp);
    outputClampedWidget.setDouble(output);
    setAngleMotor(output);

    errorWidget.setDouble(getAngleEncoder() - setpoint);
    // System.out.println("calc: " + m_controller.calculate(getAngleEncoder(),
    // setpoint));
    // System.out.println("Output: " + output);
    // System.out.println("Setpoint: " + setpoint);
  }

  @Override
  protected double getMeasurement() {
    return getAngleEncoder();
  }

  @Override
  public void periodic() {
    if (isAngleReset != angleResetWidget.getBoolean(false)) {
      isAngleReset = angleResetWidget.getBoolean(false);
      if (isAngleReset) {
        resetAngleEncoder();
      }
    }

    // speakerAngleSetpoint = speakerAngleWidget.getDouble(0);
    // ampAngleSetpoint = ampAngleWidget.getDouble(0);

    kPVal = kPWidget.getDouble(Constants.Angler.DEFAULT_PID_kP);
    kIVal = kIWidget.getDouble(Constants.Angler.DEFAULT_PID_kI);
    kDVal = kDWidget.getDouble(Constants.Angler.DEFAULT_PID_kD);

    getController().setP(kPVal);

    // commented out in case someone sets the P and the I values on the dashboard
    // getController().setI(kIVal);
    // getController().setD(kDVal);

    m_angleMotor.configure(
      new SparkMaxConfig().apply(
        new SoftLimitConfig()
          .forwardSoftLimitEnabled(!softLimitOverride.getBoolean(false))
          .reverseSoftLimitEnabled(!softLimitOverride.getBoolean(false))),
      ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    setpoint1 = setpoint1Widget.getDouble(0);
    setpoint2 = setpoint2Widget.getDouble(0);

    angleSetpoint = angleSetpointWidget.getDouble(0);
    setSetpoint(angleSetpoint);

    positiveClamp = positiveClampWidget.getDouble(Constants.Angler.DEFAULT_POSITIVE_CLAMP);
    negativeClamp = negativeClampWidget.getDouble(Constants.Angler.DEFAULT_NEGATIVE_CLAMP);

    super.periodic();

    angleEncoderWidget.setDouble(getAngleEncoder());
  }
}