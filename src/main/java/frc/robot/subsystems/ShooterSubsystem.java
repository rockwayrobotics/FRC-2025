package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  private final SparkMax m_leftFlywheel;
  private final SparkMax m_rightFlywheel;

  private final SparkClosedLoopController m_leftPID, m_rightPID;

  private DigitalInput m_shooterTopSensor;

  public boolean shooterStaged;

  public GenericEntry flywheelSpeedWidget;
  public GenericEntry flywheelWaitWidget;

  GenericEntry shooterTopSensorWidget;
  GenericEntry leftFlywheelWidget;
  GenericEntry rightFlywheelWidget;
  GenericEntry leftRPMWidget;
  GenericEntry rightRPMWidget;

  GenericEntry FlywheelSetRPMWidget;
  GenericEntry FlywheelScaleWidget;

  public double webFlywheelSpeed;
  public double webFlywheelWait;
  public double flywheelRPMSetpoint;

  private double m_scale = 1;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("Shooter");

  /** Creates a new HookSubsystem. */
  public ShooterSubsystem(int leftFlywheel, int rightFlywheel) {
    m_leftFlywheel = new SparkMax(Constants.CAN.LEFT_FLYWHEEL, MotorType.kBrushless);
    m_rightFlywheel = new SparkMax(Constants.CAN.RIGHT_FLYWHEEL, MotorType.kBrushless);
    m_leftPID = m_leftFlywheel.getClosedLoopController();
    m_rightPID = m_rightFlywheel.getClosedLoopController();

    // TODO: It seemed safest to reset to safe first and then avoid persisting the
    // changes through power cycle?
    m_leftFlywheel.configure(
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .inverted(true)
            .smartCurrentLimit(40)
            .apply(new ClosedLoopConfig().p(0.00008).velocityFF(0.00019)),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightFlywheel.configure(
        new SparkMaxConfig()
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(40)
            .apply(new ClosedLoopConfig().p(0.00008).velocityFF(0.00018)),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // m_rightFlywheel.follow(m_leftFlywheel, true);

    m_shooterTopSensor = new DigitalInput(Constants.Digital.SHOOTER_TOP_SENSOR);

    shooterTopSensorWidget = dashboardTab.addPersistent("Shooter Staged Sensor", false).getEntry();
    // leftFlywheelWidget = dashboardTab.addPersistent("Left Flywheel Speed",
    // 0).getEntry();
    // rightFlywheelWidget = dashboardTab.addPersistent("Right Flywheel Speed",
    // 0).getEntry();
    FlywheelSetRPMWidget = dashboardTab.addPersistent("Flywheel Set RPM Setpoint", 5000).getEntry();
    FlywheelScaleWidget = dashboardTab.addPersistent("Flywheel Scale", 1).getEntry();
    flywheelSpeedWidget = dashboardTab.addPersistent("Flywheel Speed", 0.5).getEntry();
    flywheelWaitWidget = dashboardTab.addPersistent("Flywheel Wait To Ramp", 1).getEntry();
    leftRPMWidget = dashboardTab.add("Left Flywheel RPM", 0).getEntry();
    rightRPMWidget = dashboardTab.add("Right Flywheel RPM", 0).getEntry();
  }

  public double getLeftFlywheelVelocity() {
    return m_leftFlywheel.getEncoder().getVelocity();
  }

  public double getRightFlywheelVelocity() {
    return m_rightFlywheel.getEncoder().getVelocity();
  }

  public double getAverageFlywheelVelocity() {
    return (getLeftFlywheelVelocity() + getRightFlywheelVelocity()) / 2;
  }

  // public void setFlywheelsScale(double scale) {
  // m_scale = scale;
  // }

  public void instantStopFlywheels() {
    m_leftPID.setReference(0, ControlType.kVelocity);
    m_rightPID.setReference(0, ControlType.kVelocity);
  }

  public void setFlywheels(double m_pow) {
    if (m_pow == 0) {
      m_leftFlywheel.stopMotor();
      m_rightFlywheel.stopMotor();
    } else {
      m_leftPID.setReference(m_pow * flywheelRPMSetpoint, ControlType.kVelocity);
      m_rightPID.setReference(m_pow * flywheelRPMSetpoint, ControlType.kVelocity);
    }
    // m_leftFlywheel.set(m_pow * m_scale);
    // m_rightFlywheel.set(m_pow);
  }

  public boolean isNoteStaged() {
    return !m_shooterTopSensor.get();
  }

  public boolean atSpeed() {
    return getLeftFlywheelVelocity() >= flywheelRPMSetpoint - 50
        && getRightFlywheelVelocity() >= flywheelRPMSetpoint - 50;
  }

  @Override
  public void periodic() {

    // SmartDashboard.putNumber("Encoder revolutions", getAngleEncoder());

    // this one broke some stuff, look at it later
    // dashboardTab.add("Encoder revolutions", getAngleEncoder());

    shooterTopSensorWidget.setBoolean(isNoteStaged());
    // leftFlywheelWidget.setDouble(m_leftFlywheel.get());
    // rightFlywheelWidget.setDouble(m_rightFlywheel.get());

    leftRPMWidget.setDouble(getLeftFlywheelVelocity());
    rightRPMWidget.setDouble(getRightFlywheelVelocity());

    webFlywheelSpeed = flywheelSpeedWidget.getDouble(0.5);
    webFlywheelWait = flywheelWaitWidget.getDouble(1);

    flywheelRPMSetpoint = FlywheelSetRPMWidget.getDouble(5000);
    m_scale = FlywheelScaleWidget.getDouble(1);

    shooterStaged = isNoteStaged();
  }
}