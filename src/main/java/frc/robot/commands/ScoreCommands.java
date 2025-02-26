package frc.robot.commands;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.Constants;
import frc.robot.RobotTracker;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.drive.Drive;

public class ScoreCommands {
  public static Command score(Drive drive, Elevator elevator, Chute chute) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    IntegerPublisher piState = nt.getIntegerTopic(Constants.NT.SENSOR_MODE).publish();
    FloatSubscriber corner = nt.getFloatTopic(Constants.NT.CORNERS).subscribe(0, PubSubOption.periodic(0.01));
    AtomicReference<Float> cornerTime = new AtomicReference<Float>(0.0f);
    ParallelRaceGroup cancellableGroup = new ParallelRaceGroup();
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
            Commands.runOnce(() -> piState.set(RobotTracker.getInstance().getScoringState().sensorState.piValue())),
            Commands.waitUntil(() -> {
              float[] corners = corner.readQueueValues();
              if (corners.length > 0) {
                cornerTime.set(corners[corners.length - 1]);
                return true;
              }
              return false;
            }),
            Commands.runOnce(() -> {
              Optional<Double> leftEncoderDistance = drive.getLeftPositionAtTime(cornerTime.get());
              leftEncoderDistance.ifPresentOrElse(distance -> {
                RobotTracker.getInstance().getScoringState().setCornerDistance(distance);
              }, () -> cancellableGroup.addCommands(Commands.runOnce(() -> {
              })));
            }),
            Commands.waitUntil(() -> {
              return RobotTracker.getInstance().getScoringState().readyToShoot(drive.getLeftPositionMeters());
            }),
            Commands.run(() -> {
              chute.startShooting();
            }).withTimeout(2.0),
            Commands.runOnce(() -> {
              chute.stopShooting();
            })
        )
    );
    command.addRequirements(drive, elevator, chute);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      RobotTracker.getInstance().getScoringState().reset();
      chute.stopShooting();
      // FIXME: Reset? Detect if coral was shot?
    });
  }
}
