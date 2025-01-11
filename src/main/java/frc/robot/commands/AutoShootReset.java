package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/** Four piece auto from the middle of the field, starting on the red alliance.
 * Different for each alliance so we don't smack into the truss first
 */
public class AutoShootReset extends SequentialCommandGroup {
    DrivebaseSubsystem m_drivebase;
    IntakeSubsystem m_intake;
    LedSubsystem m_led;

    public AutoShootReset(DrivebaseSubsystem drivebase, IntakeSubsystem intake, LedSubsystem led){
        m_drivebase = drivebase;
        m_intake = intake;
        m_led = led;

        // addRequirements(m_drivebase, m_intake, m_led);

        FailFastTimeoutGroup sequence = new FailFastTimeoutGroup()
                .then(new InstantCommand(() -> m_drivebase.setDrivebaseIdle(IdleMode.kBrake)))
                .then(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Red)))
                .then(new InstantCommand(() -> m_intake.setBelt(-0.7)))
                .then(new WaitCommand(0.1))
                .then(new InstantCommand(() -> m_intake.setBelt(0)));

        this.addCommands(sequence);
    }
}