package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class ShootSequenceWebAdj extends SequentialCommandGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led; 

  private ShootSequenceWebAdj(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    addRequirements(m_shooter, m_intake, m_led);

    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Green)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(m_shooter.webFlywheelSpeed)));
    this.addCommands(new WaitCommand(m_shooter.webFlywheelWait));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(1)));
    this.addCommands(new WaitCommand(1));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(0)));
    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));
  }
  
  public static Command create(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    return new ShootSequenceWebAdj(shooter, intake, led).finallyDo((boolean interrupted) -> {
      shooter.setFlywheels(0);
    });
  }
}
