package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotTracker;
import frc.robot.subsystems.drive.Drive;

public class DriveRotate extends Command {

  private Drive drive;
  private double angle;
  private Rotation2d targetRotation;
  private double direction;

  public DriveRotate(Drive drive, double angle) {
    this.drive = drive;
    this.angle = angle;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    var rotation = RobotTracker.getInstance().getEstimatedPose().getRotation();
    targetRotation = rotation.rotateBy(Rotation2d.fromDegrees(angle));
    direction = Math.signum(angle);

    System.out.println("move by angle: " + angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var rotation = RobotTracker.getInstance().getEstimatedPose().getRotation();
    var radiff = Math.abs(rotation.minus(targetRotation).getRadians());
    var speed = 1.4;
    if (radiff < Units.degreesToRadians(30)) {
      speed = radiff * radiff * 5;
    }
    drive.setTankDrive(new ChassisSpeeds(0, 0, speed * direction));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
    var rotation = RobotTracker.getInstance().getEstimatedPose().getRotation();
    System.out.println("end, moved to " + rotation.getDegrees());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    var rotation = RobotTracker.getInstance().getEstimatedPose().getRotation();
    System.out.println("wrapped angle:" + rotation.getDegrees());
    System.out.println("diff:" + rotation.minus(targetRotation).getDegrees());
    return Math.abs(rotation.minus(targetRotation).getDegrees()) < 1;
  }
}