package frc.robot.commands;

import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.FloatArrayEntry;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.FloatSubscriber;
import edu.wpi.first.networktables.FloatTopic;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.Tuner;
import frc.robot.Constants;
import frc.robot.RobotTracker;
import frc.robot.ScoringState.SensorState;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.drive.Drive;

public class ScoreCommandsOnlyDrive {
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

  public static class TestScoreCommandState extends ScoreCommandState {
    public boolean fakeCornerTriggered = false;
    public boolean scoreNowTriggered = false;

    @Override
    public void reset() {
      super.reset();
      fakeCornerTriggered = false;
      scoreNowTriggered = false;
    }
  }

  static final Tuner testScoreDriveSpeedMetersPerSec = new Tuner("TestScore/drive_speed_meters_per_sec", 0.2, true);
  static final Tuner testScoreWallDistanceMeters = new Tuner("TestScore/distance_along_wall_meters", 0.4, true);
  static final Tuner testScoreShootSpinDuration = new Tuner("TestScore/shoot_spin_duration_seconds", 3, true);

  public static final double SCORING_EPSILON_METERS = 0.25;

  /**
   * Distance in meters between selected ToF sensor and center of far reef bar,
   * assuming front sensors are used for going forwards and back sensors for
   * scoring while reversing.
   * 
   * @return distance in meters
   */
  public static double farDistance(Constants.ToFSensorLocation location) {
    switch (location) {
      case FRONT_LEFT:
        return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS + Constants.ToFSensor.FRONT_LEFT.getX()
            - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      case FRONT_RIGHT:
        return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS + Constants.ToFSensor.FRONT_RIGHT.getX()
            - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      // case BACK_LEFT:
      // return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS -
      // Constants.ToFSensor.BACK_LEFT.getX()
      // + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      // case BACK_RIGHT:
      // return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS -
      // Constants.ToFSensor.BACK_RIGHT.getX()
      // + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      default:
        return 0;
    }
  }

  public static double nearDistance(Constants.ToFSensorLocation location) {
    switch (location) {
      case FRONT_LEFT:
        return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS + Constants.ToFSensor.FRONT_LEFT.getX()
            - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      case FRONT_RIGHT:
        return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS + Constants.ToFSensor.FRONT_RIGHT.getX()
            - Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      // case BACK_LEFT:
      // return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS -
      // Constants.ToFSensor.BACK_LEFT.getX()
      // + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      // case BACK_RIGHT:
      // return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS -
      // Constants.ToFSensor.BACK_RIGHT.getX()
      // + Constants.Chute.CHUTE_CENTER_X_POSITION_METERS;
      default:
        return 0;
    }
  }

  public static double getTargetWallDistance(Constants.ReefBar reefBar, Constants.ToFSensorLocation sensorState) {
    int backOrForwards = 1;
    // if (sensorState == SensorState.BACK_LEFT || sensorState ==
    // SensorState.BACK_RIGHT) {
    // backOrForwards = -1;
    // }

    switch (reefBar) {
      case FAR:
        return backOrForwards * farDistance(sensorState);
      default:
        return backOrForwards * nearDistance(sensorState);
    }
  }

  public static Command score(Drive drive, Chute chute, Constants.ReefBar reefBar) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoublePublisher speedTopic = nt.getDoubleTopic(Constants.NT.SPEED).publish();
    StringPublisher tofTopic = nt.getStringTopic(Constants.NT.TOF_MODE).publish();
    FloatArrayTopic cornerTopic = nt.getFloatArrayTopic(Constants.NT.CORNERS);
    ParallelRaceGroup cancellableGroup = new ParallelRaceGroup();
    ScoreCommandState commandState = new ScoreCommandState();
    // FIXME make this not atomic and cringe
    final AtomicReference<Constants.ToFSensorLocation> sensorLocation = new AtomicReference<>(Constants.ToFSensorLocation.NONE_SELECTED);

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

        Commands.sequence(
            Commands.runOnce(() -> {
              speedTopic.set(drive.getLeftVelocityMetersPerSec());
              if (chute.getPivotGoalRads() > 0) {
                tofTopic.set("left");
                sensorLocation.set(Constants.ToFSensorLocation.FRONT_LEFT);
              } else {
                tofTopic.set("right");
                sensorLocation.set(Constants.ToFSensorLocation.FRONT_RIGHT);
              }
            }),
            Commands.waitUntil(() -> commandState.isValid),
            Commands.runOnce(() -> {
              Optional<Double> leftEncoderDistance = drive.getLeftPositionAtTime(commandState.cornerTimestamp);
              leftEncoderDistance.ifPresentOrElse(distance -> {
                //var scoringState = RobotTracker.getInstance().getScoringState();
                commandState.cornerDistance = distance;
                commandState.targetLeftEncoder = distance
                    + getTargetWallDistance(reefBar, sensorLocation.get()) * Math.cos(commandState.angle);
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
    command.addRequirements(drive);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      tofTopic.set("none");
      speedTopic.set(Constants.Drive.SCORING_SPEED);
      //piState.set(new double[] { SensorState.NONE.piValue(), Constants.Drive.SCORING_SPEED });
      commandState.reset();
      RobotTracker.getInstance().getScoringState().reset();
      chute.stopShooting();
      // FIXME: Reset? Detect if coral was shot?
    });
  }

  public static Command testScoreOnlyDrive(Drive drive, Chute chute, BooleanSupplier fakeCornerTrigger,
      BooleanSupplier shootNowTrigger) {
    double driveSpeed = ScoreCommandsOnlyDrive.testScoreDriveSpeedMetersPerSec.get();
    double wallDistanceMeters = ScoreCommandsOnlyDrive.testScoreWallDistanceMeters.get();
    double shootSpinDuration = ScoreCommandsOnlyDrive.testScoreShootSpinDuration.get();

    if (Math.abs(driveSpeed) > 0.5) {
      return Commands.runOnce(() -> {
        System.err.println("Rejected test score command, drive speed " + driveSpeed + " is too fast");
      });
    }

    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    //DoubleArrayPublisher piState = nt.getDoubleArrayTopic(Constants.NT.SENSOR_MODE).publish();
    FloatArrayTopic cornerTopic = nt.getFloatArrayTopic(Constants.NT.CORNERS);
    ParallelRaceGroup cancellableGroup = new ParallelRaceGroup();
    TestScoreCommandState commandState = new TestScoreCommandState();
    // We never stop listening to this
    nt.addListener(cornerTopic, EnumSet.of(Kind.kValueAll), networkTableEvent -> {
      float[] results = networkTableEvent.valueData.value.getFloatArray();
      commandState.cornerTimestamp = results[0];
      // Angle is in radians
      commandState.angle = results[1];
      commandState.isValid = true;
    });

    Command command = Commands.sequence(
        Commands.runOnce(() -> {
          System.out.println("TestScore: drive.stop");
          drive.stop();
        }),
        Commands.waitUntil(() -> {
          double speed = drive.getLeftVelocityMetersPerSec();
          System.out.println("TestScore: ready to drive");
          return (Math.abs(speed) < 0.05);
        }),
        Commands.race(
            Commands.run(() -> {
              drive.setTankDrive(new ChassisSpeeds(driveSpeed, 0, 0));
            }),
            Commands.sequence(
                Commands.waitUntil(() -> {
                  double speed = drive.getLeftVelocityMetersPerSec();
                  return (Math.abs(driveSpeed - speed) < 0.1);
                }),
                Commands.runOnce(() -> {
                  System.out.println("TestScore: Setting PiState");
                  // piState.set(new double[] { RobotTracker.getInstance().getScoringState().sensorState.piValue(),
                  //     commandState.speeds.getLast() });
                }),
                Commands.waitUntil(() -> {
                  commandState.fakeCornerTriggered = fakeCornerTrigger.getAsBoolean();
                  commandState.scoreNowTriggered = shootNowTrigger.getAsBoolean();
                  return commandState.isValid || commandState.fakeCornerTriggered || commandState.scoreNowTriggered;
                }),
                Commands.runOnce(() -> {
                  System.out.println("TestScore: Finished waiting for Pi");
                  if (commandState.scoreNowTriggered) {
                    System.out.println("TestScore: Score now processing");
                    // do nothing
                  } else {
                    if (commandState.fakeCornerTriggered) {
                      System.out.println("TestScore: Fake corner processing");
                      commandState.cornerTimestamp = (float) Timer.getFPGATimestamp();
                      commandState.angle = 0;
                    }
                    Optional<Double> leftEncoderDistance = drive.getLeftPositionAtTime(commandState.cornerTimestamp);
                    leftEncoderDistance.ifPresentOrElse(distance -> {
                      commandState.cornerDistance = distance;
                      commandState.targetLeftEncoder = distance
                          + wallDistanceMeters * Math.cos(commandState.angle);
                    }, () -> cancellableGroup.addCommands(Commands.runOnce(() -> {
                      System.err.println("Failed to find scoring encoder distance because we have no position data");
                    })));
                  }
                }),
                Commands.waitUntil(() -> {
                  return commandState.scoreNowTriggered || Math
                      .abs(drive.getLeftPositionMeters() - commandState.targetLeftEncoder) < SCORING_EPSILON_METERS;
                }),
                Commands.run(() -> {
                  System.out.println("TestScore: Starting to shoot");
                  chute.startShooting();
                }).withTimeout(shootSpinDuration),
                Commands.runOnce(() -> {
                  System.out.println("TestScore: Stopping shooting");
                  chute.stopShooting();
                }))),
        Commands.runOnce(() -> {
          System.out.println("TestScore: Stopping normally");
          drive.stop();
        }));
    command.addRequirements(drive);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      if (interrupted) {
        System.out.println("Stopped robot due to cancellation");
        drive.stop();
      }
      System.out.println("TestScore: Resetting");
      // piState.set(new double[] { SensorState.NONE.piValue(), Constants.Drive.SCORING_SPEED });
      commandState.reset();
      RobotTracker.getInstance().getScoringState().reset();
      chute.stopShooting();
      // FIXME: Reset? Detect if coral was shot?
    });
  }
}
