package frc.robot.commands;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class DriveUntilLoaded extends Command {

  private DrivebaseSubsystem m_drivebase;
  private IntakeSubsystem m_intake;
  private double m_speed;
  private double m_distance;
  private double m_leftBase;
  private double m_rightBase;
  private double m_leftDist;
  private double m_rightDist;

  private double m_totalDistance;

  public DriveUntilLoaded(DrivebaseSubsystem subsystem, IntakeSubsystem intake, double speed, double distance) {

    m_drivebase = subsystem;
    m_intake = intake;
    m_speed = speed;
    m_distance = distance;

    addRequirements(m_drivebase);
  }

  @Override
  public void initialize() {
    // Resets encoder values to default
    // System.out.println("Current Encoders: " + m_drivebase.getRDistance());
    // m_drivebase.resetEncoders();

    // System.out.println("After Encoders: " + m_drivebase.getRDistance());
    m_leftBase = m_drivebase.getLDistance();
    m_rightBase = m_drivebase.getRDistance();

    System.out.printf("Moving: %.3f from %.3f, %.3f%n",
      m_distance, m_leftBase, m_rightBase);
  }

  @Override
  public void execute() {
    m_drivebase.set(m_speed, 0);
    // System.out.println("Executing");
  }

  @Override
  public boolean isFinished() {
    // System.out.println("Current pos: " + Math.abs(m_drivebase.getRDistance()) + " Setpoint: " + m_distance);
    //SmartDashboard.putNumber("Auto Command Distance Travelled", m_drivebase.getRDistance());
    double m_leftDist = m_drivebase.getLDistance() - m_leftBase;
    double m_rightDist = m_drivebase.getRDistance() - m_rightBase;
    double distance = (m_leftDist + m_rightDist) / 2.0;

    m_totalDistance = distance; 

    if (m_intake.isNoteLoaded()) {
      System.out.println("Intake is loaded");
      return true;
    }

    return (Math.abs(distance) >= m_distance);
  }

  @Override
  public void end(boolean cancelled) {
    m_drivebase.stop(); // Resets the drivebase to 0, ends command
    m_drivebase.distanceDrivenAuto = Math.abs(m_totalDistance); // Sets the distance driven to the average of the two distances
    System.out.println("Distance Driven: " + m_drivebase.distanceDrivenAuto);
    System.out.printf("Moved: %.3f, %.3f%n", m_leftDist, m_rightDist);
  }
}
