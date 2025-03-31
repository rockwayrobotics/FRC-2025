package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotTracker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Tuner;

/**
 * Counter-clockwise rotation is positive.
 */
public class DriveRotate extends Command {

  private Drive drive;
  private double angle;
  private Rotation2d targetRotation;
  private PIDController pidController;

  private static Tuner kP = new Tuner("DriveRotate/kP", 0.03, true);
  private static Tuner kI = new Tuner("DriveRotate/kI", 0.0, true);
  private static Tuner kD = new Tuner("DriveRotate/kD", 0.0, true);

  public DriveRotate(Drive drive, double angle) {
    this.drive = drive;
    this.angle = angle;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.pidController = new PIDController(kP.get(), kI.get(), kD.get());
    this.pidController.enableContinuousInput(-180, 180);

    var rotation = RobotTracker.getInstance().getEstimatedPose().getRotation();
    targetRotation = rotation.rotateBy(Rotation2d.fromDegrees(angle));
    System.out.println("move by angle: " + angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var rotation = RobotTracker.getInstance().getEstimatedPose().getRotation();
    var output = pidController.calculate(rotation.getDegrees(), targetRotation.getDegrees());
    drive.setTankDrive(new ChassisSpeeds(0, 0, output));
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