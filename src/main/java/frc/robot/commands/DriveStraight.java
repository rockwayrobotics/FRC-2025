package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
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
  protected Tuner kP = new Tuner("DriveStraight/kP", 0.8, true);
  protected Tuner kI = new Tuner("DriveStraight/kI", 0.01, true);
  protected Tuner kD = new Tuner("DriveStraight/kD", 0.1, true);
  
  private double errorSum = 0;
  private double lastError = 0;
  private double maxCorrection = 0.4; // Maximum correction factor
  private double iZone = 0.05;

  public DriveStraight(double targetSpeedMetersPerSec, Drive drive) {
    this.drive = drive;
    this.targetSpeedMetersPerSec = targetSpeedMetersPerSec;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling getController() and casting to PIDController
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
    // Calculate the difference between left and right encoders
    double leftDistance = drive.getLeftPositionMeters() - startingLeftEncoder;
    double rightDistance = drive.getRightPositionMeters() - startingRightEncoder;
    
    // Calculate the error (difference between sides)
    double error = leftDistance - rightDistance;
    
    // Calculate PID terms
    double proportional = error * kP.get();
    
    // Integral term with anti-windup
    if (Math.abs(error) < iZone) {
      errorSum += error;
    } else {
      errorSum = 0;
    }
    errorSum = MathUtil.clamp(error, -1.0, 1.0);
    double integral = errorSum * kI.get();
    
    // Derivative term
    double derivative = (error - lastError) * kD.get();
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
