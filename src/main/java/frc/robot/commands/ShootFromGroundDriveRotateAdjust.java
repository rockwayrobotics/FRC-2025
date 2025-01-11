package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LedSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootFromGroundDriveRotateAdjust extends SequentialCommandGroup {
  DrivebaseSubsystem m_drivebase; 
  ShooterSubsystem m_shooter;
  IntakeSubsystem m_intake;
  LedSubsystem m_led; 


  public ShootFromGroundDriveRotateAdjust(ShooterSubsystem shooter, IntakeSubsystem intake, LedSubsystem led, DrivebaseSubsystem drivebase, double drivedistance, double angle, double drivelengthangle) {
    m_shooter = shooter;
    m_intake = intake;
    m_led = led;
    m_drivebase = drivebase; 

    addRequirements(m_shooter, m_intake, m_led, m_drivebase);

    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.FlashingOrange)));
    
    this.addCommands(new InstantCommand(() -> m_intake.setIntake(0.5)));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(1)));
    this.addCommands(new WaitCommand(0.3));
    this.addCommands(new InstantCommand(() -> m_intake.setIntake(0)));

    this.addCommands(new DriveDistance(m_drivebase, 0.5, drivedistance)); 

    this.addCommands(new AutoPullback(shooter, intake, led)); 
    
    this.addCommands(new DriveRotate(m_drivebase, angle));
    this.addCommands(new DriveDistance(m_drivebase, 0.5, drivelengthangle));

    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(1)));
    this.addCommands(new WaitUntilCommand(() -> m_shooter.atSpeed()));
    this.addCommands(new InstantCommand(() -> m_intake.setBelt(1)));
    this.addCommands(new WaitCommand(0.5));

    this.addCommands(new InstantCommand(() -> m_intake.setBelt(0)));
    this.addCommands(new InstantCommand(() -> m_shooter.setFlywheels(0)));
    this.addCommands(new InstantCommand(() -> m_led.setMode(Constants.LED.modes.Rainbow)));
  }
}
