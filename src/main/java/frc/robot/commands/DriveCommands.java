package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.2; // Volts/Sec

  private DriveCommands() {
  }

  public static Command defaultDrive(DoubleSupplier left_y, DoubleSupplier right_x, Drive drive) {
    return Commands.run(
        () -> {
          double speed;
          double rotation;
          double rotate_clockwise = right_x.getAsDouble() * Constants.Gamepads.JOY_ROTATE_SCALE;
          double speed_forward = left_y.getAsDouble() * Constants.Gamepads.JOY_SPEED_SCALE;

          if (Math.abs(speed_forward) < 0.01) {
            speed = 0;
          } else {
            speed = speed_forward;
          }

          if (Math.abs(rotate_clockwise) < 0.01) {
            rotation = 0;
          } else {
            rotation = rotate_clockwise;
          }
          drive.set(speed, rotation);
        }, drive).finallyDo((boolean interrupted) -> {
          drive.stop();
        });
  }

  public static Command auto1(Drive drive) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.MAX_SPEED_MPS / 10.0,
        Constants.Drive.MAX_ACCEL_MPSS / 10.0)
        .setKinematics(drive.getKinematics());
    // TODO: Add voltage constraint with feedforward: .addConstraint(null);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d()),
        List.of(new Translation2d(1, 0), new Translation2d(1.5, 0)), new Pose2d(2, 0, new Rotation2d()), config);

    LTVUnicycleController ltvController = new LTVUnicycleController(0.02);

    return new LTVCommand(exampleTrajectory, () -> drive.getPose(), ltvController, drive.getKinematics(),
        (Double leftMetersPerSecond, Double rightMetersPerSecond) -> {
          Logger.recordOutput("Traj/Speed", new double[] { leftMetersPerSecond, rightMetersPerSecond });
          drive.setTankDrive(drive.getKinematics()
              .toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond)));
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);
  }

  public static Command auto2(Drive drive) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.MAX_SPEED_MPS / 1.0,
        Constants.Drive.MAX_ACCEL_MPSS / 2.0)
        .setKinematics(drive.getKinematics());
    // TODO: Add voltage constraint with feedforward: .addConstraint(null);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(),
        List.of(new Translation2d(2, 0), new Translation2d(3, 0)), new Pose2d(4, 0, new Rotation2d()), config);

    LTVUnicycleController ltvController = new LTVUnicycleController(0.02);

    double kS = 0.18389;// 0.0; // 0.18607
    double kV = 2.26057;// 0.0; // 2.25536
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV);

    double kP = 1.7;
    double kI = 0.0;
    double kD = 0.0;
    PIDController leftController = new PIDController(kP, kI, kD);
    PIDController rightController = new PIDController(kP, kI, kD);

    drive.setPose(exampleTrajectory.getInitialPose());
    return new LTVCommand(exampleTrajectory, () -> drive.getPose(), ltvController, feedForward, drive.getKinematics(),
        () -> drive.getWheelSpeeds(),
        leftController,
        rightController,
        (Double leftVoltage, Double rightVoltage) -> {
          drive.runOpenLoop(leftVoltage, rightVoltage);
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);
  }

  public static Command auto3(Drive drive) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.MAX_SPEED_MPS / 10,
        Constants.Drive.MAX_ACCEL_MPSS / 10)
        .setKinematics(drive.getKinematics());
    // TODO: Add voltage constraint with feedforward: .addConstraint(null);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d()), config);

    LTVUnicycleController ltvController = new LTVUnicycleController(0.02);

    drive.setPose(exampleTrajectory.getInitialPose());
    return new LTVCommand(exampleTrajectory, () -> drive.getPose(), ltvController, drive.getKinematics(),
        (Double leftMetersPerSecond, Double rightMetersPerSecond) -> {
          drive.setTankDrive(drive.getKinematics()
              .toChassisSpeeds(new DifferentialDriveWheelSpeeds(leftMetersPerSecond, rightMetersPerSecond)));
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);
  }

  public static Command auto4(Drive drive) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.MAX_SPEED_MPS / 1,
        Constants.Drive.MAX_ACCEL_MPSS / 2)
        .setKinematics(drive.getKinematics()).addConstraint(new CentripetalAccelerationConstraint(0.5));
    // TODO: Add voltage constraint with feedforward: .addConstraint(null);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(),
        List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d()), config);

    LTVUnicycleController ltvController = new LTVUnicycleController(0.02);

    double kS = 0.18389;// 0.0; // 0.18607
    double kV = 2.26057;// 0.0; // 2.25536
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV);

    double kP = 1.7;
    double kI = 0.0;
    double kD = 0.0;
    PIDController leftController = new PIDController(kP, kI, kD);
    PIDController rightController = new PIDController(kP, kI, kD);

    drive.setPose(exampleTrajectory.getInitialPose());
    return new LTVCommand(exampleTrajectory, () -> drive.getPose(), ltvController, feedForward, drive.getKinematics(),
        () -> drive.getWheelSpeeds(),
        leftController,
        rightController,
        (Double leftVoltage, Double rightVoltage) -> {
          drive.runOpenLoop(leftVoltage, rightVoltage);
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);
  }

  public static Command toReef(Drive drive) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.MAX_SPEED_MPS / 10,
        Constants.Drive.MAX_ACCEL_MPSS / 10)
        .setKinematics(drive.getKinematics()).addConstraint(new CentripetalAccelerationConstraint(0.5));
    // TODO: Add voltage constraint with feedforward: .addConstraint(null);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(8.5, 7.5, Rotation2d.fromDegrees(180)),
        List.of(), new Pose2d(3, 4, Rotation2d.fromDegrees(-90)), config);
    // Maybe 4,6 as intermediate?

    LTVUnicycleController ltvController = new LTVUnicycleController(0.02);

    double kS = 0.18389;// 0.0; // 0.18607
    double kV = 2.26057;// 0.0; // 2.25536
    SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(kS, kV);

    double kP = 1.7;
    double kI = 0.0;
    double kD = 0.0;
    PIDController leftController = new PIDController(kP, kI, kD);
    PIDController rightController = new PIDController(kP, kI, kD);

    drive.setPose(exampleTrajectory.getInitialPose());
    return new LTVCommand(exampleTrajectory, () -> drive.getPose(), ltvController, feedForward, drive.getKinematics(),
        () -> drive.getWheelSpeeds(),
        leftController,
        rightController,
        (Double leftVoltage, Double rightVoltage) -> {
          drive.runOpenLoop(leftVoltage, rightVoltage);
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);
  }

  public static Command driveForward(Drive drive) {
    final double speed = 0.45;
    return Commands.run(() -> {
      drive.setTankDrive(new ChassisSpeeds(speed, speed, 0));
    }, drive).withTimeout(4.5).finallyDo(() -> {
      drive.stop();
    });
  }

  /**
   * Standard joystick drive, where X is the forward-backward axis (positive =
   * forward) and Z is the
   * left-right axis (positive = counter-clockwise).
   */
  public static Command arcadeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);

          // Calculate speeds
          var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);

          // Apply output
          drive.runClosedLoop(
              speeds.left * Constants.Drive.MAX_SPEED_MPS, speeds.right * Constants.Drive.MAX_SPEED_MPS);
        },
        drive);
  }

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runOpenLoop(voltage, voltage);
              velocitySamples.add(drive.getCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
