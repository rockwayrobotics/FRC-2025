package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.Side;
import frc.robot.RobotTracker;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class AutoPaths {

  private AutoPaths() {
  }

  public static Command toReef(Drive drive, Superstructure superstructure) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.MAX_SPEED_MPS / 3,
        Constants.Drive.MAX_ACCEL_MPSS / 10)
        .setKinematics(RobotTracker.getInstance().getDriveKinematics())
        .addConstraint(new CentripetalAccelerationConstraint(0.5));
    // TODO: Add voltage constraint with feedforward: .addConstraint(null);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(8.5, 7.5, Rotation2d.fromDegrees(180)),
        List.of(), new Pose2d(3, 4, Rotation2d.fromDegrees(-90)), config);
    // Maybe 4,6 as intermediate?

    // LTVUnicycleController ltvController = new LTVUnicycleController(0.02);
    // LTVUnicycleController ltvController = new
    // LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0),
    // VecBuilder.fill(1.0, 2.0), 0.02, config.getMaxVelocity());
    LTVUnicycleController ltvController = new LTVUnicycleController(VecBuilder.fill(0.125, 0.25, 4.0),
        VecBuilder.fill(1.0, 2.0), 0.02, config.getMaxVelocity());

    double kP = 1.7;
    double kI = 0.0;
    double kD = 0.0;
    PIDController leftController = new PIDController(kP, kI, kD);
    PIDController rightController = new PIDController(kP, kI, kD);

    drive.setPose(exampleTrajectory.getInitialPose());
    return new LTVCommand(exampleTrajectory, () -> RobotTracker.getInstance().getEstimatedPose(), ltvController,
        drive.getFeedForward(), RobotTracker.getInstance().getDriveKinematics(),
        () -> drive.getWheelSpeeds(),
        leftController,
        rightController,
        (Double leftVoltage, Double rightVoltage) -> {
          drive.runOpenLoop(leftVoltage, rightVoltage);
        }, (pose) -> {
          drive.setPose(pose);
        }, drive);
  }

  public static Command midFarRightL2(Drive drive, Superstructure superstructure) {
    TrajectoryConfig config = new TrajectoryConfig(Constants.Drive.MAX_SPEED_MPS / 3,
        Constants.Drive.MAX_ACCEL_MPSS / 10)
        .setKinematics(RobotTracker.getInstance().getDriveKinematics())
        .addConstraint(new CentripetalAccelerationConstraint(0.5));
    // TODO: Add voltage constraint with feedforward: .addConstraint(null);
    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(5.389, 2.824, Rotation2d.fromDegrees(180)),
        List.of(), new Pose2d(3, 4, Rotation2d.fromDegrees(180)), config);
    // Maybe 4,6 as intermediate?

    // LTVUnicycleController ltvController = new LTVUnicycleController(0.02);
    // LTVUnicycleController ltvController = new
    // LTVUnicycleController(VecBuilder.fill(0.0625, 0.125, 2.0),
    // VecBuilder.fill(1.0, 2.0), 0.02, config.getMaxVelocity());
    LTVUnicycleController ltvController = new LTVUnicycleController(VecBuilder.fill(0.125, 0.25, 4.0),
        VecBuilder.fill(1.0, 2.0), 0.02, config.getMaxVelocity());

    double kP = 1.7;
    double kI = 0.0;
    double kD = 0.0;
    PIDController leftController = new PIDController(kP, kI, kD);
    PIDController rightController = new PIDController(kP, kI, kD);

    drive.setPose(exampleTrajectory.getInitialPose());
    LTVCommand path1 = new LTVCommand(exampleTrajectory, () -> RobotTracker.getInstance().getEstimatedPose(),
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

    return Commands.sequence(path1, Commands.runOnce(() -> superstructure.gotoSetpoint(CoralLevel.L2, Side.RIGHT)),
        Commands.runOnce(() -> superstructure.startShooting()));
  }
}
