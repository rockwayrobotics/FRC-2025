package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveRotate extends Command {

  private DrivebaseSubsystem m_drivebase;
  private double m_angle;

  public DriveRotate(DrivebaseSubsystem subsystem, double angle) {

    m_drivebase = subsystem;
    m_angle = angle;

    addRequirements(m_drivebase);
  }

  @Override
  public void initialize() {
    // Resets encoder values to default
    m_drivebase.zeroGyro();

    System.out.println("Moving: " + m_angle);
  }

  @Override
  public void execute() {
    if (m_angle < 0)
      m_drivebase.set(0, -0.5);
    else
      m_drivebase.set(0, 0.5);
    // System.out.println("Executing");
  }

  @Override
  public boolean isFinished() {
    // System.out.println("Current pos: " + Math.abs(m_drivebase.getAngle()) + " Setpoint: " + Math.abs(m_angle));
    //SmartDashboard.putNumber("Auto Command Distance Travelled", m_drivebase.getRDistance());
    return (Math.abs(m_drivebase.getAngle()) >= Math.abs(m_angle));
  }

  @Override
  public void end(boolean cancelled) {
    System.out.println("Moved.");
    m_drivebase.stop(); // Resets the drivebase to 0, ends command
  }
}
