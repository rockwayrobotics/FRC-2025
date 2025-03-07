package frc.robot.commands;

import java.util.List;
import edu.wpi.first.math.Vector;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
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

  private static final double trajectoryMaxVelocity = Constants.Drive.MAX_SPEED_MPS / 3;
  private static final double trajectoryMaxAcceleration = Constants.Drive.MAX_ACCEL_MPSS / 10;

  private static final double trajectoryMaxCentripetalAcceleration = 0.5;

  // adjustable auto wait time
  private final static Tuner autoWaitTime = new Tuner("AutoWaitTime", 0.0, false);

  private AutoPaths() {
  }

  // TODO: Add voltage constraint with feedforward: .addConstraint(null);

  public static Command midFarRightL2(Drive drive, Superstructure superstructure) {
    TrajectoryConfig config = new TrajectoryConfig(trajectoryMaxVelocity, trajectoryMaxAcceleration)
        .setKinematics(RobotTracker.getInstance().getDriveKinematics())
        .addConstraint(new CentripetalAccelerationConstraint(trajectoryMaxCentripetalAcceleration));

    Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
        new Pose2d(7.464, 4.000, Rotation2d.fromDegrees(180)),
        List.of(), new Pose2d(5.389, 2.824, Rotation2d.fromDegrees(180)), config);

    LTVUnicycleController ltvController = new LTVUnicycleController(LTV_qelems, LTV_relems, LTV_dt,
        config.getMaxVelocity());
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

    return Commands.sequence(Commands.waitSeconds(autoWaitTime.get()), path1,
        Commands.runOnce(() -> superstructure.gotoSetpoint(CoralLevel.L2, Side.RIGHT)),
        Commands.waitSeconds(1),
        Commands.runOnce(() -> superstructure.startShooting()));
  }
}
