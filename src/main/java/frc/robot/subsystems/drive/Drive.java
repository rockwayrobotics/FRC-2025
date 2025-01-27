// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDrive differentialDrive;
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(
      Constants.Drive.TRACK_WIDTH_METERS);
  // FIXME: kS and kV are feed-forward constants that should be measured
  // empirically and
  // should vary between simulator and real.
  private final double kS = 0.0;
  private final double kV = 0.0;
  private final SysIdRoutine sysId;
  private final DifferentialDrivePoseEstimator poseEstimator = new DifferentialDrivePoseEstimator(kinematics,
      new Rotation2d(), 0, 0, new Pose2d());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;
  private double scale = 1.0;

  // FIXME: Stop publishing twice to save bandwidth
  // Publish RobotPose for AdvantageScope
  StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/Robot/Pose", Pose2d.struct).publish();

  // Publish RobotPose for Shuffleboard.
  ShuffleboardTab dashboard = Shuffleboard.getTab("Drivebase");
  Field2d field = new Field2d();

  public Drive(DriveIO io, GyroIO gyroIO) {
    this.io = io;
    this.gyroIO = gyroIO;
    this.differentialDrive = io.getDifferentialDrive();
    dashboard.add("Field2d", field);

    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
        (ChassisSpeeds speeds) -> runClosedLoop(speeds),
        new PPLTVController(0.02, Constants.Drive.MAX_SPEED_MPS),
        Constants.PathPlanner.CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    // This is for logging pathfinding, by setting up our own pathfinder that
    // intercepts
    // the calls to log them.
    // Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          // FIXME: Logging
          // Logger.recordOutput(
          // "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // Configure SysId
    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            Voltage.ofBaseUnits(0.2, Volts).div(Time.ofBaseUnits(1, Seconds)),
            Voltage.ofBaseUnits(3, Volts),
            Time.ofBaseUnits(10, Seconds),
            (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism(
            (voltage) -> runOpenLoop(voltage.in(Volts), voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", inputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // FIXME: This is used for simulator, because we pretend the gyro is not
      // connected
      // FIXME: Fix this so that the gyro is actually simulated and we avoid checking
      // in periodic if the gyro is connected.
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = kinematics.toTwist2d(
          getLeftPositionMeters() - lastLeftPositionMeters,
          getRightPositionMeters() - lastRightPositionMeters);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      lastLeftPositionMeters = getLeftPositionMeters();
      lastRightPositionMeters = getRightPositionMeters();
    }

    // Update odometry
    poseEstimator.update(rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters());
    var robotPose = poseEstimator.getEstimatedPosition();
    field.setRobotPose(robotPose);
    robotPosePublisher.set(robotPose);
  }

  /**
   * Sets the drive and rotate speeds.
   * 
   * @param scale double from 0 to 1
   */
  public void setScale(double scale) {
    this.scale = scale;
  }

  /** Runs the drive at the desired velocity. */
  public void runClosedLoop(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /** Runs the drive at the desired left and right velocities. */
  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    Logger.recordOutput("Drive/LeftSetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightSetpointMetersPerSec", rightMetersPerSec);

    double leftFFVolts = kS * Math.signum(leftMetersPerSec) + kV * leftMetersPerSec;
    double rightFFVolts = kS * Math.signum(rightMetersPerSec) + kV * rightMetersPerSec;
    io.setVelocity(leftMetersPerSec, rightMetersPerSec, leftFFVolts, rightFFVolts);
  }

  /** Runs the drive in open loop. */
  public void runOpenLoop(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * This was the method used for setting speed/rotation for manual driving
   * before.
   */
  public void set(double speed, double rotation) {
    differentialDrive.curvatureDrive(speed * scale, rotation * scale, true);
  }

  /** Stops the drive. */
  public void stop() {
    runOpenLoop(0.0, 0.0);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(
        rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters(), pose);
  }

  /**
   * Adds a vision measurement to the pose estimator.
   *
   * @param visionPose The pose of the robot as measured by the vision camera.
   * @param timestamp  The timestamp of the vision measurement in seconds.
   */
  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  /** Returns the position of the left wheels in meters. */
  @AutoLogOutput
  public double getLeftPositionMeters() {
    return inputs.leftPositionMeters;
  }

  /** Returns the position of the right wheels in meters. */
  @AutoLogOutput
  public double getRightPositionMeters() {
    return inputs.rightPositionMeters;
  }

  /** Returns the velocity of the left wheels in meters/second. */
  @AutoLogOutput
  public double getLeftVelocityMetersPerSec() {
    return inputs.leftVelocityMetersPerSec;
  }

  /** Returns the velocity of the right wheels in meters/second. */
  @AutoLogOutput
  public double getRightVelocityMetersPerSec() {
    return inputs.rightVelocityMetersPerSec;
  }

  /** Returns the average velocity in meters/second. */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityMetersPerSec + inputs.rightVelocityMetersPerSec) / 2.0;
  }
}
