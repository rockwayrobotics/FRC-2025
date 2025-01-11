package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootSequenceFullAuto extends SequentialCommandGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led; 


  public ShootSequenceFullAuto(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    addRequirements(m_shooter, m_intake, m_led);

    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Yellow)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(1)));
    this.addCommands(new WaitCommand(1));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(1)));
    this.addCommands(new WaitCommand(0.5));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(0)));
    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));
  }
}
