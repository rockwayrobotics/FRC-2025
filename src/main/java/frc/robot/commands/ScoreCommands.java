package frc.robot.commands;

import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
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

  static final Tuner testScoreElevatorHeightMm = new Tuner("TestScore/elevator_height_mm",
      Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM, true);
  static final Tuner testScoreChuteAngleDegrees = new Tuner("TestScore/chute_angle_degrees", Units.degreesToRadians(60),
      true);
  static final Tuner testScoreDriveSpeedMetersPerSec = new Tuner("TestScore/drive_speed_meters_per_sec", 0.2, true);
  static final Tuner testScoreWallDistanceMeters = new Tuner("TestScore/distance_along_wall_meters", 0.4, true);
  static final Tuner testScoreShootSpinDuration = new Tuner("TestScore/shoot_spin_duration_seconds", 3, true);

  public static final double SCORING_EPSILON_METERS = 0.25;
  // FIXME commented out piState stuff because it doesnt work rn 
  public static Command score(Drive drive, Superstructure superstructure) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
   // DoubleArrayPublisher piState = nt.getDoubleArrayTopic(Constants.NT.SENSOR_MODE).publish();
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
          superstructure.setElevatorGoalHeightMillimeters(
              RobotTracker.getInstance().getScoringState().reefHeight.elevatorHeight());
        }),
        Commands.runOnce(() -> {
          superstructure.setChutePivotGoalRads(RobotTracker.getInstance().getScoringState().pivotRadians());
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
              // piState.set(new double[] { RobotTracker.getInstance().getScoringState().sensorState.piValue(),
              //     commandState.speeds.getLast() });
            }),
            Commands.waitUntil(() -> commandState.isValid),
            Commands.runOnce(() -> {
              Optional<Double> leftEncoderDistance = drive.getLeftPositionAtTime(commandState.cornerTimestamp);
              leftEncoderDistance.ifPresentOrElse(distance -> {
                var scoringState = RobotTracker.getInstance().getScoringState();
                commandState.cornerDistance = distance;
                commandState.targetLeftEncoder = distance
                    + scoringState.getTargetWallDistance() * Math.cos(commandState.angle);
              }, () -> cancellableGroup.addCommands(Commands.runOnce(() -> {
                System.err.println("Failed to find scoring encoder distance because we have no position data");
              })));
            }),
            Commands.waitUntil(() -> {
              return Math.abs(drive.getLeftPositionMeters() - commandState.targetLeftEncoder) < SCORING_EPSILON_METERS;
            }),
            Commands.run(() -> {
              superstructure.startShooting();
            }).withTimeout(2.0),
            Commands.runOnce(() -> {
              superstructure.stopShooting();
            })));
    command.addRequirements(drive, superstructure);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      // piState.set(new double[] { SensorState.NONE.piValue(), Constants.Drive.SCORING_SPEED });
      commandState.reset();
      RobotTracker.getInstance().getScoringState().reset();
      superstructure.stopShooting();
      // FIXME: Reset? Detect if coral was shot?
    });
  }

  public static Command testScore(Drive drive, Superstructure superstructure, BooleanSupplier fakeCornerTrigger, BooleanSupplier shootNowTrigger) {
    double elevatorHeightMm = ScoreCommands.testScoreElevatorHeightMm.get();
    double chuteAngleDegrees = ScoreCommands.testScoreChuteAngleDegrees.get();
    double driveSpeed = ScoreCommands.testScoreDriveSpeedMetersPerSec.get();
    double wallDistanceMeters = ScoreCommands.testScoreWallDistanceMeters.get();
    double shootSpinDuration = ScoreCommands.testScoreShootSpinDuration.get();

    if (elevatorHeightMm < Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
      return Commands.runOnce(() -> {
        System.err.println("Rejected test score command, height " + elevatorHeightMm + " is too low");
      });
    } else if (chuteAngleDegrees > 90 || chuteAngleDegrees < -90) {
      return Commands.runOnce(() -> {
        System.err.println("Rejected test score command, chute angle " + chuteAngleDegrees + " is not valid");
      });
    } else if (Math.abs(driveSpeed) > 0.5) {
      return Commands.runOnce(() -> {
        System.err.println("Rejected test score command, drive speed " + driveSpeed + " is too fast");
      });
    }

    double chuteAngleRads = Units.degreesToRadians(chuteAngleDegrees);

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
        return (Math.abs(speed) < 0.05);
      }),
      Commands.runOnce(() -> {
        System.out.println("TestScore: setElevatorGoal");
        superstructure.setElevatorGoalHeightMillimeters(elevatorHeightMm);
      }),
      Commands.waitUntil(() -> {
        return Math.abs(superstructure.getElevatorHeightMillimeters() - elevatorHeightMm) < 10;
      }),
      Commands.runOnce(() -> {
        System.out.println("TestScore: setChuteGoal");
        superstructure.setChutePivotGoalRads(chuteAngleRads);
      }),
      Commands.waitUntil(() -> {
        boolean ready = Math.abs(superstructure.getPivotAngleRads() - chuteAngleRads) < Units.degreesToRadians(5);
        if (ready) {
          System.out.println("TestScore: ready to drive");
        }
        return ready;
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
            //    commandState.speeds.getLast() });
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
            return commandState.scoreNowTriggered || Math.abs(drive.getLeftPositionMeters() - commandState.targetLeftEncoder) < SCORING_EPSILON_METERS;
          }),
          Commands.run(() -> {
            System.out.println("TestScore: Starting to shoot");
            superstructure.startShooting();
          }).withTimeout(shootSpinDuration),
          Commands.runOnce(() -> {
            System.out.println("TestScore: Stopping shooting");
            superstructure.stopShooting();
          })
        )
      ),
      Commands.runOnce(() -> {
        System.out.println("TestScore: Stopping normally");
        drive.stop();
      })
    );
    command.addRequirements(drive, superstructure);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      if (interrupted) {
        System.out.println("Stopped robot due to cancellation");
        drive.stop();
      }
      System.out.println("TestScore: Resetting");
      //piState.set(new double[] { SensorState.NONE.piValue(), Constants.Drive.SCORING_SPEED });
      commandState.reset();
      RobotTracker.getInstance().getScoringState().reset();
      superstructure.stopShooting();
      // FIXME: Reset? Detect if coral was shot?
    });
  }
}
