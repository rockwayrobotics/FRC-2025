package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotTracker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Tuner;

public class DriveStraight extends Command {
  private final Drive drive;
  protected Pose2d startingPose;
  protected double targetSpeedMetersPerSec;
  protected double startingLeftEncoder;
  protected double startingRightEncoder;

  // Four methods, 0-3.
  protected Tuner driveStraightMethod = new Tuner("DriveStraight/Method", 1, true);

  protected Tuner kP_orig = new Tuner("DriveStraight/MethodOrig/kP", 0.8, true);
  protected Tuner kI_orig = new Tuner("DriveStraight/MethodOrig/kI", 0.01, true);
  protected Tuner kD_orig = new Tuner("DriveStraight/MethodOrig/kD", 0.1, true);

  protected Tuner kP_method1 = new Tuner("DriveStraight/Method1/kP", 0.1, true);
  protected Tuner kI_method1 = new Tuner("DriveStraight/Method1/kI", 0.0, true);
  protected Tuner kD_method1 = new Tuner("DriveStraight/Method1/kD", 0.0, true);

  protected Tuner kP_method2 = new Tuner("DriveStraight/Method2/kP", 0.1, true);

  private PIDController straightController = new PIDController(kP_method1.get(), kI_method1.get(), kD_method1.get());

  private double errorSum = 0;
  private double lastError = 0;
  private double maxCorrection = 0.4; // Maximum correction factor
  private double iZone = 0.05;

  public DriveStraight(double targetSpeedMetersPerSec, Drive drive) {
    this.drive = drive;
    this.targetSpeedMetersPerSec = targetSpeedMetersPerSec;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling getController() and casting to
    // PIDController
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.startingPose = RobotTracker.getInstance().getEstimatedPose();
    this.startingLeftEncoder = this.drive.getLeftPositionMeters();
    this.startingRightEncoder = this.drive.getRightPositionMeters();

    // Reset PID values
    errorSum = 0;
    lastError = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int method = (int) driveStraightMethod.get();
    if (method == 1) {
      execute_method1();
    } else if (method == 2) {
      execute_method2();
    } else if (method == 3) {
      execute_method3();
    } else {
      execute_method_orig();
    }
  }

  private void execute_method1() {
    // Get encoder positions
    double leftPosition = drive.getLeftPositionMeters() - startingLeftEncoder;
    double rightPosition = drive.getRightPositionMeters() - startingRightEncoder;

    // Calculate difference between sides
    double error = leftPosition - rightPosition;

    // Calculate correction
    double correction = straightController.calculate(error, 0);

    // Apply speeds with correction
    drive.setTankDriveSpeeds(targetSpeedMetersPerSec + correction, targetSpeedMetersPerSec - correction);
  }

  private void execute_method2() {
    // Calculate heading error
    double currentHeading = RobotTracker.getInstance().getEstimatedPose().getRotation().getRadians();
    double headingError = startingPose.getRotation().getRadians() - currentHeading;

    // Apply correction (simple proportional control)
    double correction = headingError * kP_method2.get();

    // Apply to motors with correction
    drive.setTankDriveSpeeds(targetSpeedMetersPerSec - correction, targetSpeedMetersPerSec + correction);
  }

  private void execute_method3() {
    double leftPosition = Math.abs(drive.getLeftPositionMeters() - startingLeftEncoder);
    double rightPosition = Math.abs(drive.getRightPositionMeters() - startingRightEncoder);

    // Prevent division by zero
    if (leftPosition < 0.1 && rightPosition < 0.1) {
      drive.setTankDriveSpeeds(targetSpeedMetersPerSec, targetSpeedMetersPerSec);
      return;
    }

    // Calculate ratio between sides (which side is going faster)
    double ratio;
    if (leftPosition > rightPosition) {
      ratio = rightPosition / leftPosition;
      drive.setTankDriveSpeeds(targetSpeedMetersPerSec * ratio, targetSpeedMetersPerSec);
    } else {
      ratio = leftPosition / rightPosition;
      drive.setTankDriveSpeeds(targetSpeedMetersPerSec, targetSpeedMetersPerSec * ratio);
    }
  }

  private void execute_method_orig() {
    /*
    // If we don't slow down fast enough, this may be useful
    double leftVelocityMetersPerSec = drive.getLeftVelocityMetersPerSec();
    double rightVelocityMetersPerSec = drive.getRightVelocityMetersPerSec();

    if (Math.abs(leftVelocityMetersPerSec - rightVelocityMetersPerSec) > 0.5) {
      double averageSpeed = (leftVelocityMetersPerSec + rightVelocityMetersPerSec) / 2;
      drive.setTankDriveSpeeds(averageSpeed, averageSpeed);
      this.startingLeftEncoder = this.drive.getLeftPositionMeters();
      this.startingRightEncoder = this.drive.getRightPositionMeters();
      return;
    }
    */

    // Calculate the difference between left and right encoders
    double leftDistance = drive.getLeftPositionMeters() - startingLeftEncoder;
    double rightDistance = drive.getRightPositionMeters() - startingRightEncoder;

    // Calculate the error (difference between sides)
    double error = leftDistance - rightDistance;

    // Calculate PID terms
    double proportional = error * kP_orig.get();

    // Integral term with anti-windup
    if (Math.abs(error) < iZone) {
      errorSum += error;
    } else {
      errorSum = 0;
    }
    errorSum = MathUtil.clamp(error, -1.0, 1.0);
    double integral = errorSum * kI_orig.get();

    // Derivative term
    double derivative = (error - lastError) * kD_orig.get();
    lastError = error;

    // Calculate the correction value
    double correction = proportional + integral + derivative;

    // Limit the correction to prevent overcorrection
    correction = MathUtil.clamp(correction, -maxCorrection, maxCorrection);

    // Apply the correction - if left is ahead, slow it down and speed up right
    double leftSpeed = targetSpeedMetersPerSec - correction;
    double rightSpeed = targetSpeedMetersPerSec + correction;

    // Set the motors
    drive.setTankDriveSpeeds(leftSpeed, rightSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // This command will run until interrupted or another condition is met
    // You could add a distance-based finish condition here
    return false;
  }
}
