package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequenceFullNoFlywheels extends ParallelRaceGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led; 


  private ShootSequenceFullNoFlywheels(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    addRequirements(m_shooter, m_intake, m_led);
    Command main = Commands.sequence(
      Commands.runOnce(() -> m_led.setMode(Constants.LED.modes.Yellow)),
      // Commands.runOnce(() -> m_shooter.setFlywheels(1)),
      //Commands.waitSeconds(0.8),
      Commands.waitUntil(() -> m_shooter.getLeftFlywheelVelocity() >= m_shooter.flywheelRPMSetpoint - 50 
      &&  m_shooter.getRightFlywheelVelocity() >= m_shooter.flywheelRPMSetpoint - 50),     
      Commands.runOnce(() -> m_intake.setBelt(1)),
      Commands.waitSeconds(1)
    ).finallyDo((interrupted) -> {
      m_intake.setBelt(0);
      m_shooter.setFlywheels(0);
    });
    
    Command LEDindicator = Commands.sequence(
      Commands.waitUntil(() -> m_shooter.isNoteStaged()),
      Commands.waitUntil(() -> !m_shooter.isNoteStaged()),
      Commands.runOnce(() -> m_led.setMode(Constants.LED.modes.FlashingGreen)),
      Commands.waitSeconds(2)
    ).finallyDo((interrupted) -> m_led.setMode(Constants.LED.modes.Rainbow));

    this.addCommands(main, LEDindicator);
  }

  public static Command create(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    return new ShootSequenceFullNoFlywheels(shooter, intake, led).finallyDo((boolean interrupted) -> {
      shooter.setFlywheels(0);
    });
  }
}
