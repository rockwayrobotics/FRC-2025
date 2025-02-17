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
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.drive.Drive;

public class DockingCommand extends Command {
  private TimeInterpolatableBuffer<Double> positions;
  private ToFSide side;
  private Drive drive;
  private Chute chute;
  private double speedMetersPerSec;

  public static final double MIN_SPEED = 0.3; // m/s
  public static final double MAX_SPEED = 0.5; // m/s

  // Distance from corner to right in front of post
  public static final double METERS_TO_NEAR_POST = 0.3061;
  public static final double METERS_TO_FAR_POST = 0.6347;

  public DockingCommand(ToFSide side, Drive drive, Chute chute) {
    this.side = side;
    this.drive = drive;
    this.chute = chute;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    positions = TimeInterpolatableBuffer.createDoubleBuffer(5);

    speedMetersPerSec = MathUtil.clamp((drive.getLeftVelocityMetersPerSec() + drive.getRightVelocityMetersPerSec()) / 2, MIN_SPEED,
        MAX_SPEED);
    System.out.printf("Speed: %.2f%n", speedMetersPerSec);
    RobotTracker.getInstance().startRecordingToF(side, speedMetersPerSec);
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
        // FIXME: This has the obvious problem that we may have already passed our point.
        // FIXME: This also has the problem that if we are on an angle, we may want to shoot when we are
        // not precisely in front of the bar, so we can hit the bar with the coral on an angle.
        System.out.printf("Position at corner: %.2f, now: %.2f%n", positionAtCorner.get(), positionNow);
        double parallelDistanceFromCorner = METERS_TO_NEAR_POST + Constants.ToFSensor.FRONT_LEFT.getX()
            - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
        double nonParallelDistanceTravelled = positionNow - positionAtCorner.get();
        double wallAngle = RobotTracker.getInstance().getFrontCornerDetector().getWallAngle(speedMetersPerSec);
        double nonParallelDesiredDistance = parallelDistanceFromCorner / Math.cos(wallAngle);
        double difference = nonParallelDesiredDistance - nonParallelDistanceTravelled;
        System.out.printf("Need to travel %.3f (%.3f - %.3f) (%.2f degrees)%n", difference, nonParallelDesiredDistance,
            nonParallelDistanceTravelled, Degrees.convertFrom(wallAngle, Radians));
        if (difference <= 0) {
          chute.shoot();
          this.cancel();
          return;
        }
      }
    }

    // Tank drive is the only way to set a speed in m/s at the moment, set(speed) is -1 to 1.
    drive.setTankDrive(new ChassisSpeeds(speedMetersPerSec, speedMetersPerSec, 0));
    //drive.set(speed, 0);
    /*
     * Not sure what to do with back corner detector
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
