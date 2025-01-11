package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.Constants;

public class LoadShooterSequence extends SequentialCommandGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led;

  public LoadShooterSequence(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    addRequirements(m_intake, m_shooter, m_led);

    this.addCommands(new InstantCommand(() ->m_led.setMode(Constants.LED.modes.Blue)));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0.5)));
    this.addCommands(new WaitCommand(3.5));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(-0.5)));
    this.addCommands(new WaitCommand(0.5));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));
    this.addCommands(new InstantCommand(() ->m_led.setMode(Constants.LED.modes.Rainbow)));
  }
}
