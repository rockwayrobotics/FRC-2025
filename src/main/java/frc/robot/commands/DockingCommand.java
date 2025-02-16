package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotTracker;
import frc.robot.RobotTracker.ToFSide;
import frc.robot.subsystems.drive.Drive;

public class DockingCommand extends Command {
  private TimeInterpolatableBuffer<Double> positions;
  private ToFSide side;
  private Drive drive;
  private double speed;

  public static final double MIN_SPEED = 0.3; // m/s
  public static final double MAX_SPEED = 0.5; // m/s

  // Distance from corner to right in front of post
  public static final double METERS_TO_NEAR_POST = 0.3061;
  public static final double METERS_TO_FAR_POST = 0.6347;

  public DockingCommand(ToFSide side, Drive drive) {
    this.side = side;
    this.drive = drive;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    positions = TimeInterpolatableBuffer.createDoubleBuffer(5);
    RobotTracker.getInstance().startRecordingToF(side);

    speed = MathUtil.clamp((drive.getLeftVelocityMetersPerSec() + drive.getRightVelocityMetersPerSec()) / 2, MIN_SPEED,
        MAX_SPEED);
    System.out.printf("Speed: %.2f%n", speed);
  }

  @Override
  public void execute() {
    double now = Timer.getFPGATimestamp();
    double positionNow = drive.getLeftPositionMeters();
    positions.addSample(now, positionNow);

    Optional<Double> frontCornerTimestamp = RobotTracker.getInstance().getFrontCornerDetector().getCornerTimestamp();
    if (frontCornerTimestamp.isPresent()) {
      System.out.printf("Found front corner: %.2f%n", frontCornerTimestamp.get());
      Optional<Double> positionAtCorner = positions.getSample(frontCornerTimestamp.get());
      if (positionAtCorner.isPresent()) {
        System.out.printf("Position at corner: %.2f, now: %.2f%n", positionAtCorner.get(), positionNow);
        double parallelDistanceFromCorner = METERS_TO_NEAR_POST + Constants.ToFSensor.FRONT_LEFT.getX()
            - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        double nonParallelDistanceTravelled = positionNow - positionAtCorner.get();
        double wallAngle = RobotTracker.getInstance().getFrontCornerDetector().getWallAngle(speed);
        double nonParallelDesiredDistance = parallelDistanceFromCorner / Math.cos(wallAngle);
        double difference = nonParallelDesiredDistance - nonParallelDistanceTravelled;
        System.out.printf("Need to travel %.3f (%.3f - %.3f) (%.2f degrees)%n", difference, nonParallelDesiredDistance,
            nonParallelDistanceTravelled, Degrees.convertFrom(wallAngle, Radians));
        if (difference <= 0) {
          drive.stop();
          this.cancel();
          return;
        }
      }
    }

    drive.setTankDrive(new ChassisSpeeds(speed, speed, 0));
    //drive.set(speed, 0);
    /*
     * Optional<Double> backCornerTimestamp =
     * RobotTracker.getInstance().getBackCornerDetector().getCornerTimestamp();
     * if (backCornerTimestamp.isPresent()) {
     * System.out.printf("Found back corner: %.2f%n", backCornerTimestamp.get());
     * }
     */
  }

  @Override
  public void end(boolean interrupted) {
    RobotTracker.getInstance().stopRecordingToF();
    drive.stop();
  }
}
