package frc.robot.commands;

import static edu.wpi.first.units.Units.Seconds;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
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
import frc.robot.Constants.AlgaeLevel;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.ReefBar;
import frc.robot.Constants.Side;
import frc.robot.RobotTracker;
import frc.robot.subsystems.chuterShooter.ChuterShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.Sensors;
import frc.robot.util.TrajectoryUtils;
import frc.robot.util.Tuner;

public class AutoPaths {
  private static final double kP = 1.7;
  private static final double kI = 0.0;
  private static final double kD = 0.0;

  private static final Vector<N3> LTV_qelems = VecBuilder.fill(0.125, 0.25, 4.0);
  private static final Vector<N2> LTV_relems = VecBuilder.fill(1.0, 2.0);
  private static final double LTV_dt = 0.02;

  private static final double trajectoryMaxVelocity = Constants.Drive.MAX_SPEED_MPS;
  private static final double trajectoryMaxAcceleration = Constants.Drive.MAX_ACCEL_MPSS / 4;

  private static final double trajectoryMaxCentripetalAcceleration = 0.5;

  static final Tuner stabilizeDelayTuner = new Tuner("TroughStabilizeWaitSeconds", 0.3, true);
  static final Tuner troughShooterSpeedTuner = new Tuner("TroughShootSpeed", 0.13, true);
  static final Tuner troughDriveSpeedTuner = new Tuner("TroughDriveSpeedMetersPerSec", 0, true);
  static final Tuner troughTimeTuner = new Tuner("TroughDurationSeconds", 5, true);

  static final Tuner algaeRotateSpeed = new Tuner("AlgaeRotateSpeed", 0.5, true);
  static final Tuner algaeDriveSpeed = new Tuner("AlgaeDriveSpeed", 1, true);

  // adjustable auto wait time
  private final static Tuner autoWaitTime = new Tuner("AutoWaitTime", 0.0, false);

  // A trajectory config that ends at the configured scoring speed for
  // ScoreCommands
  private static final TrajectoryConfig autoScoreConfig = new TrajectoryConfig(1, 0.2)
      .setEndVelocity(ScoreCommandsOnlyDrive.scoringSpeedMetersPerSecond.get())
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration));

  // The forward trajectory config that applies to most autos
  private static final TrajectoryConfig config = new TrajectoryConfig(trajectoryMaxVelocity, trajectoryMaxAcceleration)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration));

  private static final TrajectoryConfig slightlyFasterConfig = new TrajectoryConfig(trajectoryMaxVelocity, 1)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration));

  private static final TrajectoryConfig fasterConfig = new TrajectoryConfig(trajectoryMaxVelocity, 2)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration));

  // The reverse trajectory config that applies to most autos
  private static final TrajectoryConfig reversedConfig = new TrajectoryConfig(trajectoryMaxVelocity,
      trajectoryMaxAcceleration)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration))
      .setReversed(true);

  private static final TrajectoryConfig fasterReversedConfig = new TrajectoryConfig(trajectoryMaxVelocity,
      2)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration))
      .setReversed(true);

  // A slow forward trajectory config
  private static final TrajectoryConfig slowConfig = new TrajectoryConfig(1, 0.2)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(0.5));

  // A slow reverse trajectory config
  private static final TrajectoryConfig slowReverseConfig = new TrajectoryConfig(1,
      0.2)
      .setKinematics(RobotTracker.getInstance().getDriveKinematics())
      .addConstraint(new CentripetalAccelerationConstraint(0.5))
      .setReversed(true);

  private AutoPaths() {
  }

  // TODO: Add voltage constraint with feedforward: .addConstraint(null);

  /**
   * Moves forward half a meter and stops at 0.5m/s. Takes 1 second.
   */
  public static Command justMove(Drive drive, Superstructure superstructure) {
    return Commands.run(() -> {
      drive.setTankDrive(new ChassisSpeeds(1, 0, 0));
    }, drive).withTimeout(Seconds.of(1)).finallyDo(() -> {
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

  private static Command runTrajectory(Trajectory trajectory, Drive drive) {
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

    return path1;
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
  private static Command runShootingTrajectory(Trajectory trajectory, CoralLevel level, Side side, Drive drive,
      Superstructure superstructure, ChuterShooter chuterShooter) {
    Command followPath = runTrajectory(trajectory, drive);

    Command command = Commands.sequence(Commands.waitSeconds(autoWaitTime.get()), followPath,
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.gotoSetpoint(level, side);
        }),
        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() && superstructure.isPivotAtGoal();
        }),
        Commands.waitSeconds(1), // Arbitrary delay to let elevator stabilize
        Commands.runOnce(() -> {
          chuterShooter.startShooting();
        }),
        Commands.waitSeconds(0.5)).finallyDo(() -> {
          chuterShooter.stopShooting();
          superstructure.setChutePivotGoalRads(
              superstructure.getPivotAngleRads() > 0 ? Units.degreesToRadians(90) : Units.degreesToRadians(-90));
          superstructure.setElevatorGoalHeightMillimeters(0);
          drive.stop();
        });
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command midFarRightL2(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
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
    return runShootingTrajectory(trajectory, CoralLevel.L2, Side.LEFT, drive, superstructure, chuterShooter);
  }

  public static Command rightNearCenterL2(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
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
    return runShootingTrajectory(trajectory, CoralLevel.L2, Side.LEFT, drive, superstructure, chuterShooter);
  }

  public static Command leftNearCenterL2(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
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
    return runShootingTrajectory(trajectory, CoralLevel.L2, Side.RIGHT, drive, superstructure, chuterShooter);
  }

  public static Command rightNearCenterTrough(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
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
    return runTroughTrajectory(trajectory, Side.LEFT, drive, superstructure, chuterShooter);
  }

  public static Command leftNearCenterTrough(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
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
    return runTroughTrajectory(trajectory, Side.RIGHT, drive, superstructure, chuterShooter);
  }

  public static Command rightNearRightTrough(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    double start_pose_x = 7.464;
    double start_pose_y = 1.000;
    double start_pose_heading_deg = 180;
    // 820mm robot width
    // 410mm from halfway
    // Using a 3 cm margin out from the AprilTag gives us (4.051750369519958,
    // 2.811041322334847).
    // (4.194038343361741, 2.7288913223348468) is earlier on the wall
    double end_pose_x = 4.194;
    double end_pose_y = 2.729;
    double end_pose_heading_deg = 150;

    List<Translation2d> interior_waypoints = List.of();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        interior_waypoints, new Pose2d(end_pose_x, end_pose_y, Rotation2d.fromDegrees(end_pose_heading_deg)), config);
    return runTroughTrajectory(trajectory, Side.LEFT, drive, superstructure, chuterShooter);
  }

  public static Command leftNearLeftTrough(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    double start_pose_x = 7.464;
    double start_pose_y = 7.050; // 1 m from wall
    double start_pose_heading_deg = 180;
    // Using a 3cm margin out from the AprilTag gives us (4.051750369519958,
    // 5.240758677665153).
    // (4.194038343361742, 5.322908677665153) is earlier on the wall
    double end_pose_x = 4.194;
    double end_pose_y = 5.323;
    double end_pose_heading_deg = 210;

    List<Translation2d> interior_waypoints = List.of();

    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        interior_waypoints, new Pose2d(end_pose_x, end_pose_y, Rotation2d.fromDegrees(end_pose_heading_deg)), config);
    return runTroughTrajectory(trajectory, Side.RIGHT, drive, superstructure, chuterShooter);
  }

  public static Command rightTestTrough(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(),
        new Pose2d(0.5, 0, new Rotation2d()), config);
    return runTroughTrajectory(trajectory, Side.LEFT, drive, superstructure, chuterShooter);
  }

  public static Command leftTestTrough(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()), List.of(),
        new Pose2d(0.5, 0, new Rotation2d()), config);
    return runTroughTrajectory(trajectory, Side.RIGHT, drive, superstructure, chuterShooter);
  }

  private static Command runTroughTrajectory(Trajectory trajectory, Side side, Drive drive,
      Superstructure superstructure, ChuterShooter chuterShooter) {
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

    Command command = Commands.sequence(Commands.waitSeconds(autoWaitTime.get()), path1,
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.gotoSetpoint(CoralLevel.L1, side);
        }),
        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() && superstructure.isPivotAtGoal();
        }),
        Commands.waitSeconds(stabilizeDelayTuner.get()), // Arbitrary delay to let elevator stabilize
        Commands.parallel(
            Commands.runOnce(() -> {
              chuterShooter.setShooterMotor(troughShooterSpeedTuner.get());
            }),
            Commands.run(() -> {
              drive.setTankDrive(new ChassisSpeeds(troughDriveSpeedTuner.get(), 0, 0));
            }).withTimeout(troughTimeTuner.get())).finallyDo(() -> {
              chuterShooter.stopShooting();
              superstructure.setChutePivotGoalRads(
                  superstructure.getPivotAngleRads() > 0 ? Units.degreesToRadians(90) : Units.degreesToRadians(-90));
              superstructure.setElevatorGoalHeightMillimeters(0);
              drive.stop();
            }));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command grabTroughAlgaeL3(Drive drive, Superstructure superstructure) {
    double start_pose_x = 4.194;
    double start_pose_y = 5.393;
    double start_pose_heading_deg = 210;

    double position_self_x = 5.890;
    double position_self_y = 6.603;
    double position_self_heading_deg = 270;

    double approach_algae_x = 5.157;
    double approach_algae_y = 5.109;
    double approach_algae_heading_deg = 120;
    // touching the wall is (5.110239999999999, 5.101418440955404)

    double grab_algae_x = 4.895;
    double grab_algae_y = 4.702;
    double grab_algae_heading_deg = 120;

    double backup_algae_x = 5.157;
    double backup_algae_y = 5.109;
    double backup_algae_heading_deg = 120;

    Trajectory startPositioningTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        List.of(), new Pose2d(position_self_x, position_self_y, Rotation2d.fromDegrees(position_self_heading_deg)),
        reversedConfig);

    Trajectory approachAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(position_self_x, position_self_y,
            Rotation2d.fromDegrees(position_self_heading_deg)),
        List.of(), new Pose2d(approach_algae_x, approach_algae_y, Rotation2d.fromDegrees(approach_algae_heading_deg)),
        config);

    Trajectory grabAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(approach_algae_x, approach_algae_y,
            Rotation2d.fromDegrees(approach_algae_heading_deg)),
        List.of(), new Pose2d(grab_algae_x, grab_algae_y, Rotation2d.fromDegrees(grab_algae_heading_deg)), slowConfig);

    Trajectory backupAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(grab_algae_x, grab_algae_y,
            Rotation2d.fromDegrees(grab_algae_heading_deg)),
        List.of(), new Pose2d(backup_algae_x, backup_algae_y, Rotation2d.fromDegrees(backup_algae_heading_deg)),
        slowReverseConfig);

    return Commands.sequence(
        runTrajectory(startPositioningTrajectory, drive),
        runTrajectory(approachAlgaeTrajectory, drive),
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.gotoAlgaeSetpoint(AlgaeLevel.L2);
        }),
        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() && superstructure.isGrabberWristAtGoal();
        }),
        Commands.runOnce(() -> {
          // Suck in algae
          superstructure.setGrabberMotor(-1);
        }),
        runTrajectory(grabAlgaeTrajectory, drive),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> {
          // stop
          superstructure.setGrabberMotor(0);
        }),
        runTrajectory(backupAlgaeTrajectory, drive),
        Commands.runOnce(() -> {
          superstructure.setElevatorGoalHeightMillimeters(20);
        }));
  }

  public static Command rightFarRightAlgaeL3(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    // Center of the reef = (4.489,4.026)
    // Start 1m from wall at our usual spot on right
    Pose2d startPose = new Pose2d(7.464, 1.000, Rotation2d.fromDegrees(180));
    // Move to 1.611m away from reef center, pointed at tag 20
    Pose2d algaePrepare = new Pose2d(5.2945, 8.05 - 5.4212, Rotation2d.fromDegrees(120));
    Pose2d flippedAlgaePrepare = TrajectoryUtils.rotatePose180(algaePrepare);
    // Move to 1.411m away from reef center, pointed at tag 20, which should be
    // basically touching wall
    Pose2d algaeLoad = new Pose2d(5.1945, 8.05 - 5.248, Rotation2d.fromDegrees(120));
    // Robot width is 0.41m, and the distance from wall to line is 0.178m, so aim to
    // have
    // robot center 0.588m from wall.
    // The point 0.588 out from the AprilTag is (4.905 + 0.294, 4.745 + 0.5092) =
    // (5.199, 5.2542).
    // However, we want to be parallel to the wall, about the wall length away (so
    // we can see the corner)
    // The wall length is 0.96m, but we can't seem to turn accurately in that
    // distance.
    // (5.199 - 0.96 cos(30deg), 5.2542 + 0.96 sin(30deg)) = (4.3676, 5.7342)
    // Try 1.5m?
    double distanceFromTag = 1.2;
    Pose2d scorePrep = new Pose2d(5.199 - distanceFromTag * Math.cos(Units.degreesToRadians(30)),
        8.05 - (5.2542 + distanceFromTag * Math.sin(Units.degreesToRadians(30))), Rotation2d.fromDegrees(30));
    Pose2d flippedScorePrep = TrajectoryUtils.rotatePose180(scorePrep);

    Trajectory algaePrepareTrajectory = TrajectoryGenerator.generateTrajectory(List.of(startPose, algaePrepare),
        slightlyFasterConfig);
    Trajectory algaeLoadTrajectory = TrajectoryGenerator.generateTrajectory(List.of(algaePrepare, algaeLoad),
        config);
    Trajectory backupTrajectory = TrajectoryGenerator.generateTrajectory(List.of(algaeLoad, algaePrepare, scorePrep),
        reversedConfig);

    var command = Commands.sequence(
        Commands.parallel(
            runTrajectory(algaePrepareTrajectory, drive),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return algaePrepare.getTranslation().getDistance(robotPose) < 0.5
                      || flippedAlgaePrepare.getTranslation().getDistance(robotPose) < 0.5;
                }),
                Commands.runOnce(() -> {
                  // Raise the elevator and grabber when we are within 0.5m of our ready position
                  superstructure.gotoAlgaeSetpoint(AlgaeLevel.L3);
                }))),

        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() &&
              superstructure.isGrabberWristAtGoal();
        }),

        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(-1);
        }),

        runTrajectory(algaeLoadTrajectory, drive),

        Commands.runOnce(() -> drive.stop()),

        Commands.race(
            Commands.waitSeconds(0.5),
            Commands.waitUntil(() -> Sensors.getInstance().getGrabberAcquired())),
        Commands.waitSeconds(0.1),

        Commands.parallel(
            Commands.runOnce(() -> {
              superstructure.setGrabberMotor(0);
            }),
            runTrajectory(backupTrajectory, drive),
            Commands.sequence(
                Commands.runOnce(() -> superstructure.gotoAlgaeSetpoint(Constants.AlgaeLevel.Score)),
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return scorePrep.getTranslation().getDistance(robotPose) < 0.5
                      || flippedScorePrep.getTranslation().getDistance(robotPose) < 0.5;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoSetpoint(CoralLevel.L3, Side.RIGHT);
                }))),

        // Commands.waitSeconds(0.5),

        ScoreCommandsOnlyDrive.score(drive, superstructure, ReefBar.NEAR, chuterShooter));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command leftNearAutoL2(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    // 1 m from wall
    Pose2d startPose = new Pose2d(7.464, 7.050, Rotation2d.fromDegrees(180));
    Pose2d endPose = new Pose2d(5.2085, 5.9086, Rotation2d.fromDegrees(210));
    Pose2d flippedPose = TrajectoryUtils.rotatePose180(endPose);
    Trajectory approachAutoScoreTrajectory = TrajectoryGenerator.generateTrajectory(startPose, List.of(), endPose,
        autoScoreConfig);

    var command = Commands.sequence(
        Commands.parallel(
            runTrajectory(approachAutoScoreTrajectory, drive),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return endPose.getTranslation().getDistance(robotPose) < 1.0
                      || flippedPose.getTranslation().getDistance(robotPose) < 1.0;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoSetpoint(CoralLevel.L2, Side.RIGHT);
                }))),
        ScoreCommandsOnlyDrive.score(drive, superstructure, ReefBar.NEAR, chuterShooter));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command rightNearAutoL2(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    // 1 m from wall
    Pose2d startPose = new Pose2d(7.464, 1.0, Rotation2d.fromDegrees(180));
    Pose2d endPose = new Pose2d(5.2085, 2.1432, Rotation2d.fromDegrees(150));
    Pose2d flippedEndPose = TrajectoryUtils.rotatePose180(endPose);
    Trajectory approachAutoScoreTrajectory = TrajectoryGenerator.generateTrajectory(startPose, List.of(), endPose,
        autoScoreConfig);

    var command = Commands.sequence(
        Commands.parallel(
            runTrajectory(approachAutoScoreTrajectory, drive),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return endPose.getTranslation().getDistance(robotPose) < 1.0
                      || flippedEndPose.getTranslation().getDistance(robotPose) < 1.0;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoSetpoint(CoralLevel.L2, Side.LEFT);
                }))),
        ScoreCommandsOnlyDrive.score(drive, superstructure, ReefBar.NEAR, chuterShooter));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command rightNearCenterAutoL2(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    // 1 m from wall
    Pose2d startPose = new Pose2d(7.464, 1.0, Rotation2d.fromDegrees(180));
    Pose2d endPose = new Pose2d(2.9, 3, Rotation2d.fromDegrees(90));
    Pose2d flippedEndPose = TrajectoryUtils.rotatePose180(endPose);
    Trajectory approachAutoScoreTrajectory = TrajectoryGenerator.generateTrajectory(startPose, List.of(), endPose,
        autoScoreConfig);

    var command = Commands.sequence(
        Commands.parallel(
            runTrajectory(approachAutoScoreTrajectory, drive),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return endPose.getTranslation().getDistance(robotPose) < 1.0
                      || flippedEndPose.getTranslation().getDistance(robotPose) < 1.0;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoSetpoint(CoralLevel.L2, Side.LEFT);
                }))),
        ScoreCommandsOnlyDrive.score(drive, superstructure, ReefBar.NEAR, chuterShooter));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command leftNearCenterAutoL2(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    // 1 m from wall
    Pose2d startPose = new Pose2d(7.464, 7.05, Rotation2d.fromDegrees(180));
    Pose2d endPose = new Pose2d(3.05, 5.2, Rotation2d.fromDegrees(-90));
    Pose2d flippedEndPose = TrajectoryUtils.rotatePose180(endPose);
    Trajectory approachAutoScoreTrajectory = TrajectoryGenerator.generateTrajectory(startPose, List.of(), endPose,
        autoScoreConfig);

    var command = Commands.sequence(
        Commands.parallel(
            runTrajectory(approachAutoScoreTrajectory, drive),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return endPose.getTranslation().getDistance(robotPose) < 1.0
                      || flippedEndPose.getTranslation().getDistance(robotPose) < 1.0;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoSetpoint(CoralLevel.L2, Side.RIGHT);
                }))),
        ScoreCommandsOnlyDrive.score(drive, superstructure, ReefBar.NEAR, chuterShooter));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command centerFarCenterDump(Drive drive, Superstructure superstructure) {
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.580, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d algaeDump = new Pose2d(5.9, yCenter, Rotation2d.fromDegrees(180.00));
    AtomicReference<Trajectory> algaeDumpTrajectory = new AtomicReference<>();
    algaeDumpTrajectory.set(TrajectoryGenerator.generateTrajectory(List.of(startPose, algaeDump),
        config));

    var command = Commands.sequence(
        runTrajectory(algaeDumpTrajectory.get(), drive),
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.setElevatorGoalHeightMillimeters(400);
          superstructure.setWristGoalRads(Units.degreesToRadians(0));
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(1),
        Commands.run(() -> {
          superstructure.setGrabberMotor(1);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(2),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          drive.stop();
        }));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command fastCenterFarDump(Drive drive, Superstructure superstructure) {
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.580, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d algaeDump = new Pose2d(5.9, yCenter, Rotation2d.fromDegrees(180.00));
    AtomicReference<Trajectory> algaeDumpTrajectory = new AtomicReference<>();
    algaeDumpTrajectory.set(TrajectoryGenerator.generateTrajectory(List.of(startPose, algaeDump),
        fasterConfig));

    var command = Commands.sequence(
        Commands.runOnce(() -> {
          superstructure.setElevatorGoalHeightMillimeters(400);
          superstructure.setWristGoalRads(Units.degreesToRadians(0));
        }),
        runTrajectory(algaeDumpTrajectory.get(), drive),
        Commands.run(() -> {
          superstructure.setGrabberMotor(1);
          drive.stop();
        }).withTimeout(0.5),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.2),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          drive.stop();
        }));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command fastCenterFarDumpPlusAlgaeGrip(Drive drive, Superstructure superstructure) {
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.580, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d algaeDump = new Pose2d(5.9, yCenter, Rotation2d.fromDegrees(180.00));
    AtomicReference<Trajectory> algaeDumpTrajectory = new AtomicReference<>();
    algaeDumpTrajectory.set(TrajectoryGenerator.generateTrajectory(List.of(startPose, algaeDump),
        fasterConfig));

    var command = Commands.sequence(
        Commands.runOnce(() -> {
          superstructure.setElevatorGoalHeightMillimeters(400);
          superstructure.setWristGoalRads(Units.degreesToRadians(0));
        }),
        runTrajectory(algaeDumpTrajectory.get(), drive),
        Commands.run(() -> {
          superstructure.setGrabberMotor(1);
          drive.stop();
        }).withTimeout(0.5),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.2),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.gotoAlgaeSetpoint(Constants.AlgaeLevel.L2);
        }).withTimeout(0.2),
        Commands.run(() -> {
          drive.set(0.2, 0);
          superstructure.setGrabberMotor(-1);
        }).withTimeout(1),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.bargePrepare();
          drive.stop();
        }));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command fastCenterFarDumpPlusAlgaeGripBargePrepare(Drive drive, Superstructure superstructure) {
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.580, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d coralDump = new Pose2d(5.9, yCenter, Rotation2d.fromDegrees(180.00));
    Trajectory coralDumpTrajectory = TrajectoryGenerator.generateTrajectory(List.of(startPose, coralDump),
        fasterConfig);
    Pose2d algaePrepare = new Pose2d(6.2, yCenter, Rotation2d.fromDegrees(180.00));
    Trajectory backUpTrajectory = TrajectoryGenerator.generateTrajectory(List.of(coralDump, algaePrepare),
        reversedConfig);
    Pose2d backupStart = backUpTrajectory.getInitialPose();
    Pose2d flippedBackupStart = TrajectoryUtils.rotatePose180(backupStart);
    Trajectory getAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(List.of(algaePrepare, coralDump),
        fasterConfig);
    Pose2d bargePrepare = new Pose2d(7.3, 6.0, Rotation2d.fromDegrees(-45));
    Trajectory gotoBargePrepareTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(algaePrepare, bargePrepare),
        fasterReversedConfig);

    var command = Commands.sequence(
        Commands.parallel(
            // Raise the elevator and hold the gripper out (coral is still held)
            Commands.runOnce(() -> {
              superstructure.setElevatorGoalHeightMillimeters(400);
              superstructure.setWristGoalRads(Units.degreesToRadians(0));
            }),
            // ... while driving to the reef in front of us
            runTrajectory(coralDumpTrajectory, drive)),
        // Stop for a short time and spin to drop coral
        Commands.run(() -> {
          superstructure.setGrabberMotor(1);
          drive.stop();
        }).withTimeout(0.5),
        // Stop spinning
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        // and stay stopped for a very short time to let coral bounce off us if
        // necessary
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.2),

        // Scored coral at this point

        // In parallel backup and drive forwards to the reef
        Commands.parallel(
            // Drive sequence
            Commands.sequence(
                // Back up ...
                runTrajectory(backUpTrajectory, drive),
                // And then go forwards again (starting to spin motors)
                Commands.parallel(
                    runTrajectory(getAlgaeTrajectory, drive),
                    Commands.runOnce(() -> superstructure.setGrabberMotor(-1)))),
            // Elevator sequence - raise into position when safe to do so
            Commands.sequence(
                Commands.waitUntil(() -> {
                  // Wait until we are far enough away to raise the elevator
                  Translation2d robotPosition = RobotTracker.getInstance().getEstimatedPose().getTranslation();
                  return Math.min(backupStart.getTranslation()
                      .getDistance(robotPosition),
                      flippedBackupStart.getTranslation().getDistance(robotPosition)) > 0.2;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoAlgaeSetpoint(Constants.AlgaeLevel.L2);
                }))),
        // Back up so we have the algae at a safe distance
        // Also stop spinning our grabber and lower our elevator
        Commands.parallel(
            runTrajectory(backUpTrajectory, drive),
            Commands.waitSeconds(1).finallyDo(() -> {
              superstructure.setGrabberMotor(0);
              superstructure.bargePrepare();
            })),

        // Head for our shot position
        runTrajectory(gotoBargePrepareTrajectory, drive),
        // Stop to stabilize
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.2),
        // Shoot!
        Commands.runOnce(() -> {
          superstructure.bargeShot();
        }),
        // Stop for the remaining auto time
        Commands.run(() -> {
          drive.stop();
        }));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command fastLeftFarDumpPlusAlgaeGripBargeShot(Drive drive, Superstructure superstructure) {
    Pose2d startPose = new Pose2d(7.2, 7.3, Rotation2d.fromDegrees(180.00));
    Pose2d coralDump = new Pose2d(5.194, 5.247, Rotation2d.fromDegrees(240.00));
    Trajectory coralDumpTrajectory = TrajectoryGenerator.generateTrajectory(List.of(startPose, coralDump),
        fasterConfig);
    Pose2d algaePrepare = new Pose2d(5.344, 5.507, Rotation2d.fromDegrees(240.00));
    Trajectory backUpTrajectory = TrajectoryGenerator.generateTrajectory(List.of(coralDump, algaePrepare),
        reversedConfig);
    Pose2d backupStart = backUpTrajectory.getInitialPose();
    Pose2d flippedBackupStart = TrajectoryUtils.rotatePose180(backupStart);
    Trajectory getAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(List.of(algaePrepare, coralDump),
        fasterConfig);
    Pose2d bargePrepare = new Pose2d(7, 6, Rotation2d.fromDegrees(0));
    Trajectory gotoBargePrepareTrajectory = TrajectoryGenerator.generateTrajectory(
        List.of(algaePrepare, bargePrepare),
        fasterReversedConfig);

    var command = Commands.sequence(
        Commands.parallel(
            // Raise the elevator and hold the gripper out (coral is still held)
            Commands.runOnce(() -> {
              superstructure.setElevatorGoalHeightMillimeters(400);
              superstructure.setWristGoalRads(Units.degreesToRadians(0));
            }),
            // ... while driving to the reef in front of us
            runTrajectory(coralDumpTrajectory, drive)),
        // Stop for a short time and spin to drop coral
        Commands.run(() -> {
          superstructure.setGrabberMotor(1);
          drive.stop();
        }).withTimeout(0.5),
        // Stop spinning
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        // and stay stopped for a very short time to let coral bounce off us if
        // necessary
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.2),

        // Scored coral at this point

        // In parallel backup and drive forwards to the reef
        Commands.parallel(
            // Drive sequence
            Commands.sequence(
                // Back up ...
                runTrajectory(backUpTrajectory, drive),
                // And then go forwards again (starting to spin motors)
                Commands.parallel(
                    runTrajectory(getAlgaeTrajectory, drive),
                    Commands.runOnce(() -> superstructure.setGrabberMotor(-1)))),
            // Elevator sequence - raise into position when safe to do so
            Commands.sequence(
                Commands.waitUntil(() -> {
                  // Wait until we are far enough away to raise the elevator
                  Translation2d robotPosition = RobotTracker.getInstance().getEstimatedPose().getTranslation();
                  return Math.min(backupStart.getTranslation()
                      .getDistance(robotPosition),
                      flippedBackupStart.getTranslation().getDistance(robotPosition)) > 0.2;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoAlgaeSetpoint(Constants.AlgaeLevel.L2);
                }))),
        // Back up so we have the algae at a safe distance
        // Also stop spinning our grabber and lower our elevator
        Commands.parallel(
            runTrajectory(backUpTrajectory, drive),
            Commands.waitSeconds(1).finallyDo(() -> {
              superstructure.setGrabberMotor(0);
              superstructure.bargePrepare();
            })),

        // Head for our shot position
        runTrajectory(gotoBargePrepareTrajectory, drive),
        // Stop for the remaining auto time
        Commands.run(() -> {
          drive.stop();
        }));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command fastCenterFarDumpPlusAlgaeGrripBargeTurn(Drive drive, Superstructure superstructure) {
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.130, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d algaeDump = new Pose2d(5.9, yCenter, Rotation2d.fromDegrees(180.00));
    AtomicReference<Trajectory> algaeDumpTrajectory = new AtomicReference<>();
    algaeDumpTrajectory.set(TrajectoryGenerator.generateTrajectory(List.of(startPose, algaeDump),
        slightlyFasterConfig));

    var command = Commands.sequence(
        Commands.runOnce(() -> {
          superstructure.setElevatorGoalHeightMillimeters(400);
          superstructure.setWristGoalRads(Units.degreesToRadians(0));
        }),
        runTrajectory(algaeDumpTrajectory.get(), drive),
        Commands.run(() -> {
          superstructure.setGrabberMotor(0.5);
          drive.stop();
        }).withTimeout(0.5),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.2),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.gotoAlgaeSetpoint(Constants.AlgaeLevel.L2);
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.3),
        Commands.run(() -> {
          drive.set(0.2, 0);
          superstructure.setGrabberMotor(-1);
        }).withTimeout(1),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
          superstructure.bargePrepare();
        }),
        new DriveRotate(drive, -135),
        Commands.run(() -> {
          // At 0.3 for 2 s, we went 1955 mm
          // We want to go 845 mm, but we are accelerating from a stop
          // so we can't just linearly interpolate.
          drive.set(0.3, 0);
        }).withTimeout(.93),
        Commands.runOnce(() -> {
          drive.stop();
        }),
        superstructure.bargeShot(),
        Commands.runOnce(() -> superstructure.gotoSetpoint(CoralLevel.AutoSafe, Side.LEFT)),
        Commands.waitSeconds(0.3),
        Commands.run(() -> drive.set(-0.5, 0)).withTimeout(0.6));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command fastLeftTroughDumpPlusAlgaeGrripBargeTurn(Drive drive, Superstructure superstructure) {
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.03, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d algaeDump = new Pose2d(4.08, yCenter, Rotation2d.fromDegrees(180.00));
    AtomicReference<Trajectory> algaeDumpTrajectory = new AtomicReference<>();
    algaeDumpTrajectory.set(TrajectoryGenerator.generateTrajectory(List.of(startPose, algaeDump),
        fasterConfig));

    var command = Commands.sequence(
        Commands.runOnce(() -> {
          superstructure.setElevatorGoalHeightMillimeters(400);
          superstructure.setWristGoalRads(Units.degreesToRadians(0));
        }),
        runTrajectory(algaeDumpTrajectory.get(), drive),
        Commands.run(() -> {
          superstructure.setGrabberMotor(0.5);
          drive.stop();
        }).withTimeout(0.5),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(0.2),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.gotoAlgaeSetpoint(Constants.AlgaeLevel.L3);
        }).withTimeout(0.4),
        Commands.run(() -> {
          drive.set(0.2, 0);
          superstructure.setGrabberMotor(-1);
        }).withTimeout(1),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
          superstructure.bargePrepare();
        }),
        new DriveRotate(drive, 105),
        Commands.run(() -> {
          // At 0.3 for 2 s, we went 1955 mm
          // We want to go 845 mm, but we are accelerating from a stop
          // so we can't just linearly interpolate.
          drive.set(0.4, 0);
        }).withTimeout(1.45),
        Commands.runOnce(() -> {
          drive.stop();
        }),
        superstructure.bargeShot(),
        Commands.runOnce(() -> superstructure.gotoSetpoint(CoralLevel.AutoSafe, Side.LEFT)),
        Commands.waitSeconds(0.3),
        Commands.run(() -> drive.set(-0.5, 0)).withTimeout(0.6));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command sideTroughDump(Drive drive, Superstructure superstructure) {
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.580, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d algaeDump = new Pose2d(4.08, yCenter, Rotation2d.fromDegrees(180.00));
    AtomicReference<Trajectory> algaeDumpTrajectory = new AtomicReference<>();
    algaeDumpTrajectory.set(TrajectoryGenerator.generateTrajectory(List.of(startPose, algaeDump),
        config));

    var command = Commands.sequence(
        runTrajectory(algaeDumpTrajectory.get(), drive),
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.setElevatorGoalHeightMillimeters(400);
          superstructure.setWristGoalRads(Units.degreesToRadians(0));
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(1),
        Commands.run(() -> {
          superstructure.setGrabberMotor(1);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(0);
        }),
        Commands.run(() -> {
          drive.stop();
        }).withTimeout(2),
        Commands.run(() -> {
          drive.set(-0.2, 0);
        }).withTimeout(1),
        Commands.runOnce(() -> {
          drive.stop();
        }));
    command.addRequirements(drive, superstructure);
    return command;
  }

  public static Command centerFarCenterAlgaeL3(Drive drive, Superstructure superstructure,
      ChuterShooter chuterShooter, boolean backupRight) {
    // Centered in front of far center reef wall
    double yCenter = 4.026;
    Pose2d startPose = new Pose2d(7.580, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d algaePrepare = new Pose2d(6.1, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d flippedAlgaePrepare = TrajectoryUtils.rotatePose180(algaePrepare);
    Pose2d algaeLoad = new Pose2d(5.9, yCenter, Rotation2d.fromDegrees(180.00));
    Pose2d scorePrep = new Pose2d(6.0, yCenter + (backupRight ? 2.0 : -2.0),
        Rotation2d.fromDegrees(backupRight ? -90 : 90));
    Pose2d flippedScorePrep = TrajectoryUtils.rotatePose180(scorePrep);
    Trajectory algaePrepareTrajectory = TrajectoryGenerator.generateTrajectory(startPose, List.of(), algaePrepare,
        config);
    Trajectory algaeLoadTrajectory = TrajectoryGenerator.generateTrajectory(algaePrepare, List.of(), algaeLoad,
        config);
    Trajectory backupTrajectory = TrajectoryGenerator.generateTrajectory(List.of(algaeLoad, algaePrepare, scorePrep),
        reversedConfig);

    var command = Commands.sequence(
        Commands.parallel(
            runTrajectory(algaePrepareTrajectory, drive),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return algaePrepare.getTranslation().getDistance(robotPose) < 0.5
                      || flippedAlgaePrepare.getTranslation().getDistance(robotPose) < 0.5;
                }),
                Commands.runOnce(() -> {
                  // Raise the elevator and grabber when we are within 0.5m of our ready position
                  superstructure.gotoAlgaeSetpoint(AlgaeLevel.L2);
                }))),

        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() && superstructure.isGrabberWristAtGoal();
        }),
        Commands.runOnce(() -> {
          superstructure.setGrabberMotor(-1);
        }),

        runTrajectory(algaeLoadTrajectory, drive),

        Commands.runOnce(() -> drive.stop()),
        Commands.race(
            Commands.waitSeconds(0.5),
            Commands.waitUntil(() -> Sensors.getInstance().getGrabberAcquired())),
        Commands.waitSeconds(0.5),

        Commands.parallel(
            Commands.runOnce(() -> {
              superstructure.setGrabberMotor(0);
            }),
            runTrajectory(backupTrajectory, drive),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  var robotPose = (RobotTracker.getInstance().getEstimatedPose().getTranslation());
                  return scorePrep.getTranslation().getDistance(robotPose) < 0.5
                      || flippedScorePrep.getTranslation().getDistance(robotPose) < 0.5;
                }),
                Commands.runOnce(() -> {
                  superstructure.gotoSetpoint(CoralLevel.L3, backupRight ? Side.LEFT : Side.RIGHT);
                }))),

        Commands.waitSeconds(0.5),

        ScoreCommandsOnlyDrive.score(drive, superstructure, ReefBar.NEAR, chuterShooter));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }

  public static Command leftFarFancy(Drive drive, Superstructure superstructure, ChuterShooter chuterShooter) {
    double start_pose_x = 7.464;
    double start_pose_y = 7.050; // 1 m from wall
    double start_pose_heading_deg = 180;
    // 0.5m away is (5.3597399999999995, 5.533565117443839)
    double approach_algae_x = 5.36;
    double approach_algae_y = 5.5336;
    double approach_algae_heading_deg = 240;
    // touching the wall is (5.110239999999999, 5.101418440955404)
    double grab_algae_x = 5.11;
    double grab_algae_y = 5.101;
    double grab_algae_heading_deg = 240;

    double backup_algae_x = 4.735;
    double backup_algae_y = 6.103;
    double backup_algae_heading_deg = 285;

    // 5.317584369519958, 5.003647423627308
    double coral1_x = 5.3175;
    double coral1_y = 5.0036;
    double coral1_heading_deg = -30;

    Trajectory approachAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(start_pose_x, start_pose_y,
            Rotation2d.fromDegrees(start_pose_heading_deg)),
        List.of(), new Pose2d(approach_algae_x, approach_algae_y, Rotation2d.fromDegrees(approach_algae_heading_deg)),
        config);

    Trajectory grabAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(approach_algae_x, approach_algae_y,
            Rotation2d.fromDegrees(approach_algae_heading_deg)),
        List.of(), new Pose2d(grab_algae_x, grab_algae_y, Rotation2d.fromDegrees(grab_algae_heading_deg)), config);

    Trajectory backupAlgaeTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(grab_algae_x, grab_algae_y,
            Rotation2d.fromDegrees(grab_algae_heading_deg)),
        List.of(), new Pose2d(backup_algae_x, backup_algae_y, Rotation2d.fromDegrees(backup_algae_heading_deg)),
        reversedConfig);

    Trajectory coral1Trajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(backup_algae_x, backup_algae_y,
            Rotation2d.fromDegrees(backup_algae_heading_deg)),
        List.of(), new Pose2d(coral1_x, coral1_y, Rotation2d.fromDegrees(coral1_heading_deg)), config);

    var command = Commands.sequence(
        runTrajectory(approachAlgaeTrajectory, drive),
        Commands.runOnce(() -> {
          drive.stop();
          superstructure.gotoAlgaeSetpoint(AlgaeLevel.L3);
        }),
        Commands.waitUntil(() -> {
          return superstructure.isElevatorAtGoal() && superstructure.isGrabberWristAtGoal();
        }),
        Commands.runOnce(() -> {
          // Suck in algae
          superstructure.setGrabberMotor(-1);
        }),
        runTrajectory(grabAlgaeTrajectory, drive),
        Commands.runOnce(() -> {
          // Suck in algae
          superstructure.setGrabberMotor(0);
        }),
        runTrajectory(backupAlgaeTrajectory, drive),
        runTrajectory(coral1Trajectory, drive));
    command.addRequirements(drive, superstructure, chuterShooter);
    return command;
  }
}
