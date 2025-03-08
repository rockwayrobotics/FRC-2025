package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.Side;
import frc.robot.RobotTracker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.Tuner;

public class AutoPaths {
  private static final double kP = 1.7;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final Vector<N3> LTV_qelems = VecBuilder.fill(0.125, 0.25, 4.0);
  private static final Vector<N2> LTV_relems = VecBuilder.fill(1.0, 2.0);
  private static final double LTV_dt = 0.02;

  private static final double trajectoryMaxVelocity = Constants.Drive.MAX_SPEED_MPS;
  private static final double trajectoryMaxAcceleration = Constants.Drive.MAX_ACCEL_MPSS / 5;

  private static final double trajectoryMaxCentripetalAcceleration = 0.5;

  static final Tuner troughSpeedTuner = new Tuner("TroughSpeed", 0.1, true);

  // adjustable auto wait time
  private final static Tuner autoWaitTime = new Tuner("AutoWaitTime", 0.0, false);

  // The single trajectory config that applies to all autos
  private static final TrajectoryConfig config = new TrajectoryConfig(trajectoryMaxVelocity, trajectoryMaxAcceleration)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration));

  private AutoPaths() {
  }

  // TODO: Add voltage constraint with feedforward: .addConstraint(null);

  /**
   * Moves forward half a meter and stops at 0.5m/s. Takes 1 second.
   */
  public static Command justMove(Drive drive, Superstructure superstructure) {
    return Commands.run(() -> {
      drive.setTankDrive(new ChassisSpeeds(0.5, 0, 0));
    }).withTimeout(Seconds.of(1)).finallyDo(() -> {
      drive.stop();
    });
  }

  public static Command pushRookies(Drive drive, Superstructure superstructure) {
    return Commands.run(() -> {
      drive.setTankDrive(new ChassisSpeeds(4, 0, 0));
    }, drive).withTimeout(Seconds.of(15)).finallyDo(() -> {
      drive.stop();
    });
  }

  /**
   * Generates a trajectory
   * 
   * @param startPose
   * @param endPose
   * @param level
   * @param side
   * @param drive
   * @param superstructure
   * @return
   */
  private static Command runTrajectory(Trajectory trajectory, CoralLevel level, Side side, Drive drive,
      Superstructure superstructure) {
    LTVUnicycleController ltvController = new LTVUnicycleController(LTV_qelems, LTV_relems, LTV_dt,
        config.getMaxVelocity());
    PIDController leftController = new PIDController(kP, kI, kD);
    PIDController rightController = new PIDController(kP, kI, kD);

    drive.setPose(trajectory.getInitialPose());
    LTVCommand path1 = new LTVCommand(trajectory, () -> RobotTracker.getInstance().getEstimatedPose(),
        ltvController,
        drive.getFeedForward(), RobotTracker.getInstance().getDriveKinematics(),
        () -> drive.getWheelSpeeds(),
        leftController,
        rightController,
        (Double leftVoltage, Double rightVoltage) -> {
          drive.runOpenLoop(leftVoltage, rightVoltage);
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);

    return Commands.sequence(Commands.waitSeconds(autoWaitTime.get()), path1,
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.gotoSetpoint(level, side);
        }),
        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() && superstructure.isPivotAtGoal();
        }),
        Commands.waitSeconds(1), // Arbitrary delay to let elevator stabilize
        Commands.runOnce(() -> {
          superstructure.startShooting();
        }),
        Commands.waitSeconds(0.5)).finallyDo(() -> {
          superstructure.stopShooting();
          superstructure.setChutePivotGoalRads(
              superstructure.getPivotAngleRads() > 0 ? Units.degreesToRadians(90) : Units.degreesToRadians(-90));
          superstructure.setElevatorGoalHeightMillimeters(0);
          drive.stop();
        });
  }

  public static Command midFarRightL2(Drive drive, Superstructure superstructure) {
    double start_pose_x = 7.464;
    double start_pose_y = 4.000;
    double start_pose_heading_deg = 210;
    double end_pose_x = 5.389;
    double end_pose_y = 2.824;
    double end_pose_heading_deg = 210;

    List<Translation2d> interior_waypoints = List.of();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        interior_waypoints, new Pose2d(end_pose_x, end_pose_y, Rotation2d.fromDegrees(end_pose_heading_deg)), config);
    return runTrajectory(trajectory, CoralLevel.L2, Side.LEFT, drive, superstructure);
  }

  public static Command rightNearCenterL2(Drive drive, Superstructure superstructure) {
    double start_pose_x = 7.464;
    double start_pose_y = 1.000;
    double start_pose_heading_deg = 180;
    double end_pose_x = 3.05;
    double end_pose_y = 3.8;
    double end_pose_heading_deg = 90;

    List<Translation2d> interior_waypoints = List.of();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        interior_waypoints, new Pose2d(end_pose_x, end_pose_y, Rotation2d.fromDegrees(end_pose_heading_deg)), config);
    return runTrajectory(trajectory, CoralLevel.L2, Side.LEFT, drive, superstructure);
  }

  public static Command leftNearCenterL2(Drive drive, Superstructure superstructure) {
    double start_pose_x = 7.464;
    double start_pose_y = 7.050; // 1 m from wall
    double start_pose_heading_deg = 180;
    double end_pose_x = 3.05;
    double end_pose_y = 4.25;
    double end_pose_heading_deg = -90;

    List<Translation2d> interior_waypoints = List.of();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        interior_waypoints, new Pose2d(end_pose_x, end_pose_y, Rotation2d.fromDegrees(end_pose_heading_deg)), config);
    return runTrajectory(trajectory, CoralLevel.L2, Side.RIGHT, drive, superstructure);
  }

  public static Command rightNearCenterTrough(Drive drive, Superstructure superstructure) {
    double start_pose_x = 7.464;
    double start_pose_y = 1.000;
    double start_pose_heading_deg = 180;
    // 820mm robot width
    // 410mm from halfway
    double end_pose_x = 3.6576 - 0.41 - 0.03;
    double end_pose_y = 3.8;
    double end_pose_heading_deg = 90;

    List<Translation2d> interior_waypoints = List.of();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        interior_waypoints, new Pose2d(end_pose_x, end_pose_y, Rotation2d.fromDegrees(end_pose_heading_deg)), config);
    return runTroughTrajectory(trajectory, Side.LEFT, drive, superstructure);
  }

  public static Command leftNearCenterTrough(Drive drive, Superstructure superstructure) {
    double start_pose_x = 7.464;
    double start_pose_y = 7.050; // 1 m from wall
    double start_pose_heading_deg = 180;
    double end_pose_x = 3.6576 - 0.41 - 0.03;
    double end_pose_y = 4.25;
    double end_pose_heading_deg = -90;

    List<Translation2d> interior_waypoints = List.of();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        interior_waypoints, new Pose2d(end_pose_x, end_pose_y, Rotation2d.fromDegrees(end_pose_heading_deg)), config);
    return runTroughTrajectory(trajectory, Side.RIGHT, drive, superstructure);
  }

  public static Command rightTestTrough(Drive drive, Superstructure superstructure) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(),
        new Pose2d(0.5, 0, new Rotation2d()), config);
    return runTroughTrajectory(trajectory, Side.LEFT, drive, superstructure);
  }

  public static Command leftTestTrough(Drive drive, Superstructure superstructure) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(),
        new Pose2d(0.5, 0, new Rotation2d()), config);
    return runTroughTrajectory(trajectory, Side.RIGHT, drive, superstructure);
  }

  private static Command runTroughTrajectory(Trajectory trajectory, Side side, Drive drive,
      Superstructure superstructure) {
    LTVUnicycleController ltvController = new LTVUnicycleController(LTV_qelems, LTV_relems, LTV_dt,
        config.getMaxVelocity());
    PIDController leftController = new PIDController(kP, kI, kD);
    PIDController rightController = new PIDController(kP, kI, kD);

    drive.setPose(trajectory.getInitialPose());
    LTVCommand path1 = new LTVCommand(trajectory, () -> RobotTracker.getInstance().getEstimatedPose(),
        ltvController,
        drive.getFeedForward(), RobotTracker.getInstance().getDriveKinematics(),
        () -> drive.getWheelSpeeds(),
        leftController,
        rightController,
        (Double leftVoltage, Double rightVoltage) -> {
          drive.runOpenLoop(leftVoltage, rightVoltage);
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);

    return Commands.sequence(Commands.waitSeconds(autoWaitTime.get()), path1,
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.gotoSetpoint(CoralLevel.L1, side);
        }),
        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() && superstructure.isPivotAtGoal();
        }),
        Commands.parallel(
            Commands.runOnce(() -> {
              superstructure.setShooterMotor(troughSpeedTuner.get());
            }),
            Commands.run(() -> {
              drive.setTankDrive(new ChassisSpeeds(0.2, 0, 0));
            }).withTimeout(2)).finallyDo(() -> {
              superstructure.stopShooting();
              superstructure.setChutePivotGoalRads(
                  superstructure.getPivotAngleRads() > 0 ? Units.degreesToRadians(90) : Units.degreesToRadians(-90));
              superstructure.setElevatorGoalHeightMillimeters(0);
              drive.stop();
            }));
  }
}
