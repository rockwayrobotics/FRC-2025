package frc.robot.commands;

import java.util.function.Supplier;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveDistance extends Command {

  private Drive drive;
  private double speed;
  private Supplier<Double> distanceSupplier;
  private double distance;
  private double leftBase;
  private double rightBase;
  private double leftDist;
  private double rightDist;

  public DriveDistance(Drive subsystem, double speed, Supplier<Double> distance) {

    drive = subsystem;
    speed = speed;
    distanceSupplier = distance;

    addRequirements(drive);
  }

  public DriveDistance(Drive subsystem, double speed, double distance) {
    this(subsystem, speed, () -> { return distance; });
  }

  @Override
  public void initialize() {
    distance = distanceSupplier.get();

    // Resets encoder values to default
    // System.out.println("Current Encoders: " + m_drivebase.getRDistance());
    // m_drivebase.resetEncoders();

    // System.out.println("After Encoders: " + m_drivebase.getRDistance());
    leftBase = drive.getLeftPositionMeters();
    rightBase = drive.getRightPositionMeters();

    System.out.printf("Moving: %.3f from %.3f, %.3f%n",
      distance, leftBase, rightBase);
  }

  @Override
  public void execute() {
    drive.set(speed, 0);
    // System.out.println("Executing");
  }

  @Override
  public boolean isFinished() {
    // System.out.println("Current pos: " + Math.abs(m_drivebase.getRDistance()) + " Setpoint: " + m_distance);
    //SmartDashboard.putNumber("Auto Command Distance Travelled", m_drivebase.getRDistance());
    leftDist = drive.getLeftPositionMeters() - leftBase;
    rightDist = drive.getRightPositionMeters() - rightBase;
    double currentDistance = (leftBase + rightBase) / 2.0;
    return (Math.abs(currentDistance) >= distance);
  }

  @Override
  public void end(boolean cancelled) {
    drive.stop(); // Resets the drivebase to 0, ends command
    System.out.printf("Moved: %.3f, %.3f%n", leftDist, rightDist);
  }
}
