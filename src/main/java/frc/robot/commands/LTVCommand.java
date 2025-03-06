package frc.robot.commands;

import static edu.wpi.first.util.ErrorMessages.requireNonNullParam;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.TrajectoryUtils;

import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

/**
 * A command that uses a LTV controller ({@link LTVController}) to follow a
 * trajectory
 * {@link Trajectory} with a differential drive.
 *
 * <p>
 * The command handles trajectory-following, PID calculations, and feedforwards
 * internally. This
 * is intended to be a more-or-less "complete solution" that can be used by
 * teams without a great
 * deal of controls expertise.
 *
 * <p>
 * Advanced teams seeking more flexibility (for example, those who wish to use
 * the onboard PID
 * functionality of a "smart" motor controller) may use the secondary
 * constructor that omits the PID
 * and feedforward functionality, returning only the raw wheel speeds from the
 * LTV controller.
 *
 * <p>
 * This class is provided by the NewCommands VendorDep
 */
public class LTVCommand extends Command {
  private final Timer m_timer = new Timer();
  private final boolean m_usePID;
  private Trajectory m_trajectory;
  private boolean m_trajectoryFlipped = false;
  private final Supplier<Pose2d> m_pose;
  private final LTVUnicycleController m_follower;
  private final SimpleMotorFeedforward m_feedforward;
  private final DifferentialDriveKinematics m_kinematics;
  private final Supplier<DifferentialDriveWheelSpeeds> m_speeds;
  private final PIDController m_leftController;
  private final PIDController m_rightController;
  private final BiConsumer<Double, Double> m_output;
  private final Consumer<Pose2d> m_resetPose;
  private DifferentialDriveWheelSpeeds m_prevSpeeds = new DifferentialDriveWheelSpeeds();
  private double m_prevLeftSpeedSetpoint; // m/s
  private double m_prevRightSpeedSetpoint; // m/s
  private double m_prevTime;

  /**
   * Constructs a new LTVCommand that, when executed, will follow the provided
   * trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to
   * 12 representing
   * units of volts.
   *
   * <p>
   * Note: The controller will *not* set the outputVolts to zero upon completion
   * of the path -
   * this is left to the user, since it is not appropriate for paths with
   * nonstationary endstates.
   *
   * @param trajectory      The trajectory to follow.
   * @param pose            A function that supplies the robot pose - use one of
   *                        the odometry classes to
   *                        provide this.
   * @param controller      The LTV controller used to follow the trajectory.
   * @param feedforward     The feedforward to use for the drive.
   * @param kinematics      The kinematics for the robot drivetrain.
   * @param wheelSpeeds     A function that supplies the speeds of the left and
   *                        right sides of the robot
   *                        drive.
   * @param leftController  The PIDController for the left side of the robot
   *                        drive.
   * @param rightController The PIDController for the right side of the robot
   *                        drive.
   * @param outputVolts     A function that consumes the computed left and right
   *                        outputs (in volts) for
   *                        the robot drive.
   * @param requirements    The subsystems to require.
   */
  @SuppressWarnings("this-escape")
  public LTVCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      LTVUnicycleController controller,
      SimpleMotorFeedforward feedforward,
      DifferentialDriveKinematics kinematics,
      Supplier<DifferentialDriveWheelSpeeds> wheelSpeeds,
      PIDController leftController,
      PIDController rightController,
      BiConsumer<Double, Double> outputVolts,
      Consumer<Pose2d> resetPose,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "LTVCommand");
    m_resetPose = resetPose;
    m_pose = requireNonNullParam(pose, "pose", "LTVCommand");
    m_follower = requireNonNullParam(controller, "controller", "LTVCommand");
    m_feedforward = feedforward;
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "LTVCommand");
    m_speeds = requireNonNullParam(wheelSpeeds, "wheelSpeeds", "LTVCommand");
    m_leftController = requireNonNullParam(leftController, "leftController", "LTVCommand");
    m_rightController = requireNonNullParam(rightController, "rightController", "LTVCommand");
    m_output = requireNonNullParam(outputVolts, "outputVolts", "LTVCommand");

    m_usePID = true;

    addRequirements(requirements);
  }

  /**
   * Constructs a new LTVCommand that, when executed, will follow the provided
   * trajectory.
   * Performs no PID control and calculates no feedforwards; outputs are the raw
   * wheel speeds from
   * the LTV controller, and will need to be converted into a usable form by the
   * user.
   *
   * @param trajectory            The trajectory to follow.
   * @param pose                  A function that supplies the robot pose - use
   *                              one of the odometry classes to
   *                              provide this.
   * @param follower              The LTV follower used to follow the trajectory.
   * @param kinematics            The kinematics for the robot drivetrain.
   * @param outputMetersPerSecond A function that consumes the computed left and
   *                              right wheel speeds.
   * @param requirements          The subsystems to require.
   */
  @SuppressWarnings("this-escape")
  public LTVCommand(
      Trajectory trajectory,
      Supplier<Pose2d> pose,
      LTVUnicycleController follower,
      DifferentialDriveKinematics kinematics,
      BiConsumer<Double, Double> outputMetersPerSecond,
      Consumer<Pose2d> resetPose,
      Subsystem... requirements) {
    m_trajectory = requireNonNullParam(trajectory, "trajectory", "LTVCommand");
    m_resetPose = resetPose;
    m_pose = requireNonNullParam(pose, "pose", "LTVCommand");
    m_follower = requireNonNullParam(follower, "follower", "LTVCommand");
    m_kinematics = requireNonNullParam(kinematics, "kinematics", "LTVCommand");
    m_output = requireNonNullParam(outputMetersPerSecond, "outputMetersPerSecond", "LTVCommand");

    m_feedforward = null;
    m_speeds = null;
    m_leftController = null;
    m_rightController = null;

    m_usePID = false;

    addRequirements(requirements);
  }

  @Override
  public void initialize() {
    m_prevTime = -1;
    DriverStation.getAlliance().ifPresent((alliance) -> {
      if (alliance.equals(DriverStation.Alliance.Red)) {
        if (!m_trajectoryFlipped) {
          m_trajectory = TrajectoryUtils.flipTrajectory(m_trajectory);
          m_trajectoryFlipped = true;
        }
      } else if (alliance.equals(DriverStation.Alliance.Blue)) {
        if (m_trajectoryFlipped) {
          m_trajectory = TrajectoryUtils.flipTrajectory(m_trajectory);
          m_trajectoryFlipped = false;
        }
      }
    });

    var initialState = m_trajectory.sample(0);

    m_resetPose.accept(initialState.poseMeters);
    m_prevSpeeds = m_kinematics.toWheelSpeeds(
        new ChassisSpeeds(
            initialState.velocityMetersPerSecond,
            0,
            initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond));
    m_prevLeftSpeedSetpoint = m_prevSpeeds.leftMetersPerSecond;
    m_prevRightSpeedSetpoint = m_prevSpeeds.rightMetersPerSecond;
    m_timer.restart();
    if (m_usePID) {
      m_leftController.reset();
      m_rightController.reset();
    }
  }

  @Override
  public void execute() {
    double curTime = m_timer.get();

    if (m_prevTime < 0) {
      m_output.accept(0.0, 0.0);
      m_prevTime = curTime;
      return;
    }

    Logger.recordOutput("Trajectory/Sample", m_trajectory.sample(curTime));
    var targetWheelSpeeds = m_kinematics.toWheelSpeeds(
        m_follower.calculate(m_pose.get(), m_trajectory.sample(curTime)));
    Logger.recordOutput("Trajectory/TargetWheelSpeeds", targetWheelSpeeds);

    double leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    double rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftOutput;
    double rightOutput;

    if (m_usePID) {
      double leftFeedforward = m_feedforward.calculateWithVelocities(m_prevLeftSpeedSetpoint, leftSpeedSetpoint);

      double rightFeedforward = m_feedforward.calculateWithVelocities(m_prevRightSpeedSetpoint, rightSpeedSetpoint);

      leftOutput = leftFeedforward
          + m_leftController.calculate(m_speeds.get().leftMetersPerSecond, leftSpeedSetpoint);

      rightOutput = rightFeedforward
          + m_rightController.calculate(
              m_speeds.get().rightMetersPerSecond, rightSpeedSetpoint);
    } else {
      leftOutput = leftSpeedSetpoint;
      rightOutput = rightSpeedSetpoint;
    }

    m_output.accept(leftOutput, rightOutput);
    m_prevSpeeds = targetWheelSpeeds;
    m_prevTime = curTime;
  }

  @Override
  public void end(boolean interrupted) {
    m_timer.stop();

    if (interrupted) {
      m_output.accept(0.0, 0.0);
    }
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_trajectory.getTotalTimeSeconds());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("leftVelocity", () -> m_prevSpeeds.leftMetersPerSecond, null);
    builder.addDoubleProperty("rightVelocity", () -> m_prevSpeeds.rightMetersPerSecond, null);
  }
}
