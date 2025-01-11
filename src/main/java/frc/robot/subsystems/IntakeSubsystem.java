package frc.robot.subsystems;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax m_beltMotor;
  private final SparkMax m_leftIntake;
  private final SparkMax m_rightIntake;

  private DigitalInput m_intakeLoadSensor; 

  public boolean intakeLoad; 
  public boolean stagedFlag; 

  private LedSubsystem m_led; 

  GenericEntry intakeLoadWidget;

  RelativeEncoder m_beltEncoder;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("Intake");

  public IntakeSubsystem(int beltMotor, int leftIntake, int rightIntake, LedSubsystem led) {
    m_beltMotor = new SparkMax(beltMotor, SparkMax.MotorType.kBrushless);
    m_leftIntake = new SparkMax(leftIntake, SparkMax.MotorType.kBrushed);
    m_rightIntake = new SparkMax(rightIntake, SparkMax.MotorType.kBrushed);
    m_beltMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake).inverted(true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftIntake.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightIntake.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast).follow(m_leftIntake, true), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_beltEncoder = m_beltMotor.getEncoder();

    m_led = led;

    m_intakeLoadSensor = new DigitalInput(Constants.Digital.INTAKE_LOAD_SENSOR);
    intakeLoadWidget = dashboardTab.addPersistent("Intake Load", false).getEntry();
  }

  public void setBelt(double m_pow) {
    // System.out.println("Belt: " + m_pow);
    m_beltMotor.set(m_pow);
  }

  public void setIntake(double m_pow) {
    // System.out.println("Intake: " + m_pow);
    m_leftIntake.set(m_pow);
  }

  public boolean isNoteLoaded() {
    return !m_intakeLoadSensor.get();
  }

  public void periodic(){
    // if (!intakeLoad && isNoteLoaded()){
    //   new OperatorPullupSensor(m_robotContainer.m_shooter, m_robotContainer.m_intake, m_robotContainer.m_led)
    //   .withTimeout(1.5).andThen(new OperatorPullback(m_robotContainer.m_shooter, m_robotContainer.m_intake, m_robotContainer.m_led))
    //   .schedule();
    // }
    if (!intakeLoad && isNoteLoaded() && m_led.getMode() != Constants.LED.modes.FlashingOrange){
      m_led.setMode(Constants.LED.modes.BreathingMagenta);
    }
    if (intakeLoad && !isNoteLoaded() && m_led.getMode() != Constants.LED.modes.FlashingOrange){
        m_led.setMode(Constants.LED.modes.Rainbow);
    }

    intakeLoad = isNoteLoaded();
    intakeLoadWidget.setBoolean(isNoteLoaded());
  }
}
