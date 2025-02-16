package frc.robot;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.CornerDetector;

public class RobotTracker {
  private static RobotTracker instance = new RobotTracker();

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.Drive.TRACK_WIDTH_METERS);
  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
      new Rotation2d(), 0, 0, new Pose2d());

  private Pose2d odometryPose = new Pose2d();
  private Pose2d estimatedPose = new Pose2d();
  private ChassisSpeeds robotVelocity = new ChassisSpeeds();

  public enum ToFSide {
    NONE,
    LEFT,
    RIGHT
  }

  private ToFSide recordingSide = ToFSide.NONE;
  private CornerDetector frontCornerDetector = null;
  private CornerDetector backCornerDetector = null;

  public Rotation2d getRotation() {
    return estimatedPose.getRotation();
  }

  public void addDriveSpeeds(double leftMetersPerSec, double rightMetersPerSec) {
    robotVelocity = kinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftMetersPerSec, rightMetersPerSec));
  }

  @AutoLogOutput(key = "RobotTracker/RobotVelocity")
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotVelocity;
  }

  @AutoLogOutput(key = "RobotTracker/FieldVelocity")
  public ChassisSpeeds getFieldVelocity() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());
  }

  @AutoLogOutput(key = "RobotTracker/OdometryPose")
  public Pose2d getOdometryPose() {
    return odometryPose;
  }

  @AutoLogOutput(key = "RobotTracker/EstimatedPose")
  public Pose2d getEstimatedPose() {
    return estimatedPose;
  }

  public static RobotTracker getInstance() {
    return instance;
  }

  public DifferentialDriveKinematics getDriveKinematics() {
    return kinematics;
  }

  public void recordOdometry(Rotation2d gyroAngle, double leftMeters, double rightMeters) {
    poseEstimator.update(gyroAngle, leftMeters, rightMeters);
    Pose2d lastOdometryPose = odometryPose;
    odometryPose = poseEstimator.getEstimatedPosition();

    // Update estimated pose with the same delta
    Twist2d poseDelta = lastOdometryPose.log(odometryPose);
    estimatedPose = estimatedPose.exp(poseDelta);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp  The timestamp of the vision measurement in seconds.
   */
  public void recordVision(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public void recordToF(double[] frontLeft, double[] backLeft, double[] frontRight, double[] backRight) {
    // System.out.printf("ToF: %.2f, %.2f, %.2f, %.2f%n", frontLeft[0], backLeft[0], frontRight[0], backRight[0]);
    double now = Timer.getFPGATimestamp();
    if (recordingSide == ToFSide.LEFT) {
      if (frontCornerDetector != null) {
        if (Math.abs(frontLeft[1] - now) < 1) {
          frontCornerDetector.recordDistance(frontLeft[0], frontLeft[1]);
        }
      }
      if (backCornerDetector != null) {
        if (Math.abs(backLeft[1] - now) < 1) {
          backCornerDetector.recordDistance(backLeft[0], backLeft[1]);
        }
      }
    } else if (recordingSide == ToFSide.RIGHT) {
      if (frontCornerDetector != null) {
        if (Math.abs(frontRight[1] - now) < 1) {
          frontCornerDetector.recordDistance(frontRight[0], frontRight[1]);
        }
      }
      if (backCornerDetector != null) {
        if (Math.abs(backRight[1] - now) < 1) {
          backCornerDetector.recordDistance(backRight[0], backRight[1]);
        }
      }
    }
  }

  public void resetPose(Pose2d pose, Rotation2d rawGyroRotation, double leftMeters, double rightMeters) {
    odometryPose = pose;
    estimatedPose = pose;
    poseEstimator.resetPosition(rawGyroRotation, leftMeters, rightMeters, pose);
  }

  /**
   * FIXME: Add side parameter?
   */
  public void startRecordingToF(ToFSide side) {
    recordingSide = side;
    // FIXME: Would be better to reset?
    // FIXME: These parameters are random
    // These values were good for 1.5 m/s speeds
    // frontCornerDetector = new CornerDetector(3, 2000, 1000, 2);
    frontCornerDetector = new CornerDetector(3, 400, 200, 2);
    // backCornerDetector = new CornerDetector(7, 0.1, 0.5, 0.05, 3);
  }

  public void stopRecordingToF() {
    recordingSide = ToFSide.NONE;
  }

  public CornerDetector getFrontCornerDetector() {
    return frontCornerDetector;
  }

  public CornerDetector getBackCornerDetector() {
    return backCornerDetector;
  }

  public void periodic() {
    // Not sure what we want to do here, but this is set up to be called in all
    // modes
    // periodically so we can do updates if needed.
  }
}
