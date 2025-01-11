package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.LedSubsystem;

public class OperatorManualLoad extends SequentialCommandGroup {
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led;

  public OperatorManualLoad(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;

    addRequirements(m_intake, m_shooter, m_led);

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0.7)));
    this.addCommands(new InstantCommand(() -> m_intake.setIntake(0.2)));
    this.addCommands(new InstantCommand(() -> m_shooter.instantStopFlywheels()));
  }
}
