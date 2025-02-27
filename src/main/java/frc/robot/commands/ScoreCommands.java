package frc.robot.commands;

import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.FloatArrayEntry;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.Constants;
import frc.robot.RobotTracker;
import frc.robot.ScoringState.SensorState;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.drive.Drive;

public class ScoreCommands {
  public static class ScoreCommandState {
    public float cornerTimestamp;
    public float angle;
    public boolean isValid;
    public double cornerDistance;
    public double targetLeftEncoder;
    public CircularBuffer<Double> speeds = new CircularBuffer<Double>(3);
    public ScoreCommandState() {
      reset();
    }

    public void reset() {
      cornerTimestamp = 0;
      angle = 0;
      isValid = false;
      cornerDistance = 0;
      targetLeftEncoder = 0;
    }
  }

  public static final double SCORING_EPSILON_METERS = 0.25;

  public static Command score(Drive drive, Elevator elevator, Chute chute) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleArrayPublisher piState = nt.getDoubleArrayTopic(Constants.NT.SENSOR_MODE).publish();
    FloatArrayTopic cornerTopic = nt.getFloatArrayTopic(Constants.NT.CORNERS);
    ParallelRaceGroup cancellableGroup = new ParallelRaceGroup();
    ScoreCommandState commandState = new ScoreCommandState();
    // We never stop listening to this
    nt.addListener(cornerTopic, EnumSet.of(Kind.kValueAll), networkTableEvent -> {
      float[] results = networkTableEvent.valueData.value.getFloatArray();
      commandState.cornerTimestamp = results[0];
      // Angle is in radians
      commandState.angle = results[1];
      commandState.isValid = true;
    });

    Command command = Commands.parallel(
        Commands.run(() -> {
          drive.setTankDrive(new ChassisSpeeds(Constants.Drive.SCORING_SPEED, 0, 0));
        }),
        Commands.runOnce(() -> {
          elevator.setGoalHeightMeters(RobotTracker.getInstance().getScoringState().reefHeight.elevatorHeight());
        }),
        Commands.runOnce(() -> {
          chute.setPivotGoalRads(RobotTracker.getInstance().getScoringState().pivotRadians());
        }),
        Commands.sequence(
            Commands.waitUntil(() -> {
              double speed = drive.getLeftVelocityMetersPerSec();
              if (Math.abs(speed) < 0.1) {
                return false;
              }
              commandState.speeds.addLast(speed);
              if (commandState.speeds.size() < 3) {
                return false;
              }
              return Math.abs(commandState.speeds.getFirst() - commandState.speeds.getLast()) < 0.01;
            }),
            Commands.runOnce(() -> {
              piState.set(new double[] {RobotTracker.getInstance().getScoringState().sensorState.piValue(), commandState.speeds.getLast()});
            }),
            Commands.waitUntil(() -> commandState.isValid),
            Commands.runOnce(() -> {
              Optional<Double> leftEncoderDistance = drive.getLeftPositionAtTime(commandState.cornerTimestamp);
              leftEncoderDistance.ifPresentOrElse(distance -> {
                var scoringState = RobotTracker.getInstance().getScoringState();
                commandState.cornerDistance = distance;
                commandState.targetLeftEncoder = distance + scoringState.getTargetWallDistance() * Math.cos(commandState.angle);
              }, () -> cancellableGroup.addCommands(Commands.runOnce(() -> {
                System.err.println("Failed to find scoring encoder distance because we have no position data");
              })));
            }),
            Commands.waitUntil(() -> {
              return Math.abs(drive.getLeftPositionMeters() - commandState.targetLeftEncoder) < SCORING_EPSILON_METERS;
            }),
            Commands.run(() -> {
              chute.startShooting();
            }).withTimeout(2.0),
            Commands.runOnce(() -> {
              chute.stopShooting();
            })));
    command.addRequirements(drive, elevator, chute);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      piState.set(new double[] {SensorState.NONE.piValue(), Constants.Drive.SCORING_SPEED});
      commandState.reset();
      RobotTracker.getInstance().getScoringState().reset();
      chute.stopShooting();
      // FIXME: Reset? Detect if coral was shot?
    });
  }
}
