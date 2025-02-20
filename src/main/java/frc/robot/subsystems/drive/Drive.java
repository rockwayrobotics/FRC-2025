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

import java.security.Timestamp;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
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
  // probably measureed correctly
  private final double realkS = 0.21124;
  private final double realkV = 2.278;
  private final double simkS = 0;
  private final double simkV = 0.227;
  private final double kS;
  private final double kV;
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
  FloatArraySubscriber tofSubscriber = NetworkTableInstance.getDefault().getFloatArrayTopic("/tof/sensors")
      .subscribe(new float[] {});
  int noDataCounter = 0;

  // Publish RobotPose for Shuffleboard.
  ShuffleboardTab dashboard = Shuffleboard.getTab("Drivebase");
  Field2d field = new Field2d();

  TimeInterpolatableBuffer<Double> positionBuffer;
  TempBeamBreak beamBreak;

  public Drive(DriveIO io, GyroIO gyroIO) {
    this.kS = realkS;
    this.kV = realkV;
    this.io = io;
    this.gyroIO = gyroIO;
    this.differentialDrive = io.getDifferentialDrive();

    // Temporary stuff for drive-by ToF measurement
    beamBreak = new TempBeamBreak();
    positionBuffer = TimeInterpolatableBuffer.createDoubleBuffer(60);

    dashboard.add("Field2d", field);

    double rElem0 = 1;
    Logger.recordOutput("VelocityControlFF", rElem0);
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        () -> kinematics.toChassisSpeeds(
            new DifferentialDriveWheelSpeeds(
                getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec())),
        // (ChassisSpeeds speeds) -> runClosedLoopNoFF(speeds),
        (ChassisSpeeds speeds) -> runClosedLoop(speeds),
        // (ChassisSpeeds speeds) -> setTankDrive(speeds),
        new PPLTVController(
            VecBuilder.fill(0.0625, 0.125, 0.5),
            VecBuilder.fill(rElem0, 2),
            0.02,
            Constants.Drive.MAX_SPEED_MPS),
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
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
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

  public void enable() {
    beamBreak.enable();
  }

  public void disable() {
    beamBreak.disable();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    io.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", inputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    beamBreak.periodic();

    float[] tofOutputs = tofSubscriber.get();
    double lastChangeTime = tofSubscriber.getLastChange() / 1000000.0;
    if (tofOutputs.length >= 5) {
      Logger.recordOutput("ToF/Timestamps", new float[] { tofOutputs[0], tofOutputs[1], tofOutputs[2] });
      Logger.recordOutput("ToF/Distance", new float[] { tofOutputs[3], tofOutputs[4] });
      Logger.recordOutput("ToF/NT_Time", lastChangeTime);
      Logger.recordOutput("ToF/FPGA", now);
      Logger.recordOutput("ToF/NT_Delta", now - lastChangeTime);
    } else {
      noDataCounter++;
      Logger.recordOutput("ToF/NoData", tofOutputs.length);
    }

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


    positionBuffer.addSample(now, getLeftPositionMeters());

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

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void runClosedLoopNoFF(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    Logger.recordOutput("Drive/LeftSetpointMetersPerSec", wheelSpeeds.leftMetersPerSecond);
    Logger.recordOutput("Drive/RightSetpointMetersPerSec", wheelSpeeds.rightMetersPerSecond);
    wheelSpeeds.desaturate(Constants.Drive.MAX_SPEED_MPS);
    differentialDrive.tankDrive(wheelSpeeds.leftMetersPerSecond / Constants.Drive.MAX_SPEED_MPS,
        wheelSpeeds.rightMetersPerSecond / Constants.Drive.MAX_SPEED_MPS, false);
  }

  /** Runs the drive at the desired velocity. */
  public void runClosedLoop(ChassisSpeeds speeds) {
    var wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    runClosedLoop(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /** Runs the drive at the desired left and right velocities. */
  public void runClosedLoop(double leftMetersPerSec, double rightMetersPerSec) {
    /*
     * // Originally this code did this:
     * double leftRadPerSec = leftMetersPerSec / wheelRadiusMeters;
     * double rightRadPerSec = rightMetersPerSec / wheelRadiusMeters;
     * Logger.recordOutput("Drive/LeftSetpointRadPerSec", leftRadPerSec);
     * Logger.recordOutput("Drive/RightSetpointRadPerSec", rightRadPerSec);
     * 
     * double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
     * double rightFFVolts = kS * Math.signum(rightRadPerSec) + kV * rightRadPerSec;
     * io.setVelocity(leftRadPerSec, rightRadPerSec, leftFFVolts, rightFFVolts);
     */

    Logger.recordOutput("Drive/LeftSetpointMetersPerSec", leftMetersPerSec);
    Logger.recordOutput("Drive/RightSetpointMetersPerSec", rightMetersPerSec);

    double leftRadPerSec = leftMetersPerSec / Constants.Drive.WHEEL_RADIUS_METERS;
    double righttRadPerSec = rightMetersPerSec / Constants.Drive.WHEEL_RADIUS_METERS;
    double leftFFVolts = kS * Math.signum(leftRadPerSec) + kV * leftRadPerSec;
    double rightFFVolts = kS * Math.signum(righttRadPerSec) + kV * righttRadPerSec;
    // FIXME: use SimpleMotorFeedForward class instead

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

  public void setTankDrive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.Drive.MAX_SPEED_MPS);
    differentialDrive.tankDrive(wheelSpeeds.leftMetersPerSecond / Constants.Drive.MAX_SPEED_MPS,
        wheelSpeeds.rightMetersPerSecond / Constants.Drive.MAX_SPEED_MPS, false);
  }

  /** Stops the drive. */
  public void stop() {
    runOpenLoop(0.0, 0.0);
  }

  public Command sysIDRunAll() {
    return sysId.quasistatic(SysIdRoutine.Direction.kReverse)
        .andThen(sysId.quasistatic(SysIdRoutine.Direction.kForward))
        .andThen(sysId.dynamic(SysIdRoutine.Direction.kReverse))
        .andThen(sysId.dynamic(SysIdRoutine.Direction.kForward));
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
