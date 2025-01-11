package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final SparkMax m_climberMotor;
  private final RelativeEncoder m_climberEncoder;

  private DigitalInput m_homeSensor;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("Climber");

  GenericEntry homeSensorEntry;
  GenericEntry climberEncoderEntry;
  GenericEntry climberSpeedEntry;
  GenericEntry softLimitOverride;

  /** Creates a new HookSubsystem. */
  public ClimberSubsystem(int climberMotor /* , int topLimitSwitch, int bottomLimitSwitch */) {
    m_climberMotor = new SparkMax(climberMotor, MotorType.kBrushless);
    SoftLimitConfig softLimitConfig = new SoftLimitConfig()
      .forwardSoftLimit(Constants.Climber.FORWARD_MAX)
      .reverseSoftLimit(Constants.Climber.REVERSE_MAX)
      .forwardSoftLimitEnabled(true)
      .reverseSoftLimitEnabled(true);
    m_climberMotor.configure(
      new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true).apply(softLimitConfig),
      ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_climberEncoder = m_climberMotor.getEncoder();
    m_climberEncoder.setPosition(0);

    m_homeSensor = new DigitalInput(Constants.Digital.CLIMB_HOME_SWITCH);

    homeSensorEntry = dashboardTab.addPersistent("Climb Home Sensor", false)
        .getEntry();

    climberEncoderEntry = dashboardTab.add("Climber Encoder", 0)
        .getEntry();

    climberSpeedEntry = dashboardTab.add("Climber Speed", 0)
        .getEntry();

    softLimitOverride = dashboardTab.add("Climber Soft Limit Override", true)
        .getEntry();

    SmartDashboard.putData("Reset Climber Encoder", new InstantCommand(() -> m_climberEncoder.setPosition(0)));
  }

  public boolean isAtHome() {
    return !m_homeSensor.get();
  }

  /**
   * Extends the hook.
   */
  public void setClimber(double m_pow) {
    // System.out.println("Climber: " + m_pow);
    if (!isAtHome() || m_pow >= 0) {
      m_climberMotor.set(m_pow);
    }
  }

  /**
   * Stops the hook.
   */
  public void stop() {
    m_climberMotor.set(0);
  }

  public void periodic() {
    if (isAtHome()) {
      if (m_climberMotor.get() < 0) {
        m_climberMotor.set(0);
      }
    }
    homeSensorEntry.setBoolean(isAtHome());
    climberEncoderEntry.setDouble(m_climberEncoder.getPosition());
    climberSpeedEntry.setDouble(m_climberMotor.get());
    m_climberMotor.configure(new SparkMaxConfig().apply(
      new SoftLimitConfig()
        .forwardSoftLimitEnabled(!softLimitOverride.getBoolean(true))
        .reverseSoftLimitEnabled(!softLimitOverride.getBoolean(true))),
      ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }
}