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
import java.util.Optional;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

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
import frc.robot.RobotTracker;
import frc.robot.util.TimestampSynchronizer;
import frc.robot.util.Tuner;

public class Drive extends SubsystemBase {
  private final DriveIO io;
  private final DriveIOInputsAutoLogged inputs = new DriveIOInputsAutoLogged();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

  private final DifferentialDrive differentialDrive;
  private final SysIdRoutine sysId;
  private Rotation2d rawGyroRotation = new Rotation2d();
  private double lastLeftPositionMeters = 0.0;
  private double lastRightPositionMeters = 0.0;
  private double scale = 1.0;

  private double leftPositionShootTarget = Double.NaN;
  private double rightPositionShootTarget = Double.NaN;
  private int shootCounter = 0;

  // FIXME: Stop publishing twice to save bandwidth
  // Publish RobotPose for AdvantageScope
  StructPublisher<Pose2d> robotPosePublisher = NetworkTableInstance.getDefault()
      .getStructTopic("/Robot/Pose", Pose2d.struct).publish();
  FloatArraySubscriber tofSubscriber = NetworkTableInstance.getDefault()
      .getFloatArrayTopic("/AdvantageKit/RealOutputs/Pi/tof/sensors")
      .subscribe(new float[] {});
  FloatArraySubscriber cornerSubscriber = NetworkTableInstance.getDefault()
      .getFloatArrayTopic("/AdvantageKit/RealOutputs/Pi/tof/corners")
      .subscribe(new float[] {});
  double tofLastChangeTime = 0;
  double cornerLastChangeTime = 0;
  int noDataCounter = 0;
  TimestampSynchronizer timestampSynchronizer = new TimestampSynchronizer();

  // Publish RobotPose for Shuffleboard.
  ShuffleboardTab dashboard = Shuffleboard.getTab("Drivebase");
  Field2d field = new Field2d();

  TimeInterpolatableBuffer<Double> leftPositionBuffer;
  TimeInterpolatableBuffer<Double> rightPositionBuffer;
  TempBeamBreak beamBreak;

  public Drive(DriveIO io, GyroIO gyroIO) {
    this.io = io;
    this.gyroIO = gyroIO;
    this.differentialDrive = io.getDifferentialDrive();

    // Temporary stuff for drive-by ToF measurement
    beamBreak = new TempBeamBreak();
    leftPositionBuffer = TimeInterpolatableBuffer.createDoubleBuffer(60);
    rightPositionBuffer = TimeInterpolatableBuffer.createDoubleBuffer(60);

    dashboard.add("Field2d", field);

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

  /**
   * Meant to be called in the Drive constructor if and when we try to re-enable
   * PathPlanner.
   */
  private void setupPathPlanner() {
    double rElem0 = 1;
    Logger.recordOutput("VelocityControlFF", rElem0);
    AutoBuilder.configure(
        () -> RobotTracker.getInstance().getEstimatedPose(),
        this::setPose,
        () -> RobotTracker.getInstance().getDriveKinematics().toChassisSpeeds(getWheelSpeeds()),
        // Could run closed loop here or add software PID
        (ChassisSpeeds speeds) -> setTankDrive(speeds),
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
  }

  public void enable() {
    beamBreak.enable();
  }

  public void disable() {
    beamBreak.disable();
  }

  private void tofDistancePeriodic(double now) {
    float[] tofOutputs = tofSubscriber.get();
    double lastChangeTime = tofSubscriber.getLastChange() / 1000000.0;
    if (lastChangeTime > tofLastChangeTime) {
      if (tofOutputs.length >= 5) {
        // FIXME: We are still taking three timestamps when we probably only want
        // tofOutputs[2]?
        timestampSynchronizer.addTimes(tofOutputs[2], lastChangeTime);
        Logger.recordOutput("ToF/Everything", new float[] { tofOutputs[0], tofOutputs[1], tofOutputs[2], tofOutputs[3],
            tofOutputs[4], (float) lastChangeTime, (float) now, (float) (now - lastChangeTime) });
        /*
         * Logger.recordOutput("ToF/Timestamps", new float[] { tofOutputs[0],
         * tofOutputs[1], tofOutputs[2] });
         * Logger.recordOutput("ToF/Distance", new float[] { tofOutputs[3],
         * tofOutputs[4] });
         * Logger.recordOutput("ToF/NT_Time", lastChangeTime);
         * Logger.recordOutput("ToF/FPGA", now);
         * Logger.recordOutput("ToF/NT_Delta", now - lastChangeTime);
         */
      } else {
        noDataCounter++;
        Logger.recordOutput("ToF/NoData", tofOutputs.length);
      }
      tofLastChangeTime = lastChangeTime;
    }
  }

  private void tofCornerPeriodic() {
    float[] cornerOutputs = cornerSubscriber.get();
    double lastChangeTime = cornerSubscriber.getLastChange() / 1000000.0;
    if (lastChangeTime > cornerLastChangeTime) {
      if (cornerOutputs.length >= 2) {
        timestampSynchronizer.addTimes(cornerOutputs[0], lastChangeTime);
        double cornerFPGATimestamp = timestampSynchronizer.toFPGATimestamp(cornerOutputs[1], lastChangeTime);
        Optional<Double> leftEncoderAtCorner = leftPositionBuffer.getSample(cornerFPGATimestamp);
        Optional<Double> rightEncoderAtCorner = rightPositionBuffer.getSample(cornerFPGATimestamp);
        Logger.recordOutput("ToF/Corners", new double[] { cornerOutputs[0], cornerOutputs[1], cornerFPGATimestamp });
        if (leftEncoderAtCorner.isPresent()) {
          double leftEncoderPosition = leftEncoderAtCorner.get();
          Logger.recordOutput("ToF/CornerPosition", leftEncoderPosition);
          leftPositionShootTarget = leftEncoderPosition + 0.35 + 0.2; // 35 cm from beam break to corner on reef, 20 cm
                                                                      // between sensors
          System.out.println("Shoot target: " + leftPositionShootTarget);
        } else {
          Logger.recordOutput("ToF/Errors", "No position for " + cornerFPGATimestamp);
        }
        if (rightEncoderAtCorner.isPresent()) {
          double rightEncoderPosition = rightEncoderAtCorner.get();
          Logger.recordOutput("ToF/RightCornerPosition", rightEncoderPosition);
          rightPositionShootTarget = rightEncoderPosition + 0.35 + 0.2; // 35 cm from beam break to corner on reef, 20
                                                                        // cm between sensors
          System.out.println("Shoot targetR: " + rightPositionShootTarget);
        } else {
          Logger.recordOutput("ToF/Errors", "No position for " + cornerFPGATimestamp);
        }

      }
      cornerLastChangeTime = lastChangeTime;
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive", inputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    beamBreak.periodic();

    double now = Timer.getFPGATimestamp();
    tofDistancePeriodic(now);
    tofCornerPeriodic();

    if (gyroInputs.connected) {
      // Use the real gyro angle
      rawGyroRotation = gyroInputs.yawPosition;
    } else {
      // FIXME: This is used for simulator, because we pretend the gyro is not
      // connected
      // FIXME: Fix this so that the gyro is actually simulated and we avoid checking
      // in periodic if the gyro is connected.
      // Use the angle delta from the kinematics and module deltas
      Twist2d twist = RobotTracker.getInstance().getDriveKinematics().toTwist2d(
          getLeftPositionMeters() - lastLeftPositionMeters,
          getRightPositionMeters() - lastRightPositionMeters);
      rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      lastLeftPositionMeters = getLeftPositionMeters();
      lastRightPositionMeters = getRightPositionMeters();
    }

    double leftPositionMeters = getLeftPositionMeters();
    double rightPositionMeters = getRightPositionMeters();
    // FIXME: Figure out backwards
    if (!Double.isNaN(leftPositionShootTarget) && leftPositionMeters > leftPositionShootTarget) {
      System.out.println("Shooting now:" + now);
      shootCounter++;
      leftPositionShootTarget = Double.NaN;
    }
    if (!Double.isNaN(rightPositionShootTarget) && rightPositionMeters > rightPositionShootTarget) {
      System.out.println("Shooting nowR:" + now);
      shootCounter++;
      rightPositionShootTarget = Double.NaN;
    }
    Logger.recordOutput("ToF/ShootCounter", shootCounter);
    leftPositionBuffer.addSample(now, leftPositionMeters);
    rightPositionBuffer.addSample(now, rightPositionMeters);

    // Update odometry
    RobotTracker.getInstance().recordOdometry(rawGyroRotation, leftPositionMeters, rightPositionMeters);
    RobotTracker.getInstance().addDriveSpeeds(getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());

    Pose2d robotPose = RobotTracker.getInstance().getEstimatedPose();
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

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        getLeftVelocityMetersPerSec(), getRightVelocityMetersPerSec());
  }

  /** Runs the drive in open loop. */
  public void runOpenLoop(double leftVolts, double rightVolts) {
    io.setVoltage(leftVolts, rightVolts);
  }

  /**
   * This method is only used for defaultDrive.
   */
  public void set(double speed, double rotation) {
    differentialDrive.curvatureDrive(speed * scale, rotation * scale, true);
  }

  /**
   * This method is used for non-PID trajectory following.
   *
   * @param speeds in m/s
   */
  public void setTankDrive(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = RobotTracker.getInstance().getDriveKinematics().toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Constants.Drive.MAX_SPEED_MPS);
    Logger.recordOutput("Traj/TankDriveSpeed",
        new double[] { wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond });
    Logger.recordOutput("Traj/TankDriveNormalized",
        new double[] { wheelSpeeds.leftMetersPerSecond / Constants.Drive.MAX_SPEED_MPS,
            wheelSpeeds.rightMetersPerSecond / Constants.Drive.MAX_SPEED_MPS });
    differentialDrive.tankDrive(wheelSpeeds.leftMetersPerSecond / Constants.Drive.MAX_SPEED_MPS,
        wheelSpeeds.rightMetersPerSecond / Constants.Drive.MAX_SPEED_MPS, false);
  }

  /** Stops the drive. */
  public void stop() {
    io.setVoltage(0.0, 0.0);
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

  public SimpleMotorFeedforward getFeedForward() {
    return io.getFeedForward();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    RobotTracker.getInstance().resetPose(pose, rawGyroRotation, getLeftPositionMeters(), getRightPositionMeters());
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

  public Optional<Double> getLeftPositionAtTime(double fpgaTime) {
    return leftPositionBuffer.getSample(fpgaTime);
  }

  public Optional<Double> getRightPositionAtTime(double fpgaTime) {
    return rightPositionBuffer.getSample(fpgaTime);
  }

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
