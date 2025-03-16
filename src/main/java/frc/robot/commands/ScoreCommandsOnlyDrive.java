package frc.robot.commands;

import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.util.ShotCalc;
import frc.robot.util.Tuner;
import frc.robot.Constants;
import frc.robot.subsystems.chuterShooter.ChuterShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class ScoreCommandsOnlyDrive {
  public static class ScoreCommandState {
    public AtomicReference<Float> cornerTimestamp;
    public AtomicReference<Float> cornerDistanceMm;
    public AtomicReference<Double> distanceTimestamp;
    public AtomicReference<Double> distanceDistanceMm;
    public double lastProcessedTimestamp;
    public boolean isValid;
    public ShotCalc shotCalc;

    public ScoreCommandState() {
      reset();
    }

    public void reset() {
      cornerTimestamp.set(0f);
      cornerDistanceMm.set(0f);
      distanceTimestamp.set(0d);
      distanceDistanceMm.set(0d);
      lastProcessedTimestamp = 0d;
      isValid = false;
      shotCalc = null;
    }
  }

  static final Tuner chuteTofDistanceMeters = new Tuner("Score/chute_tof_distance_meters", 0.26, true);
  static final Tuner scoringSpeedMetersPerSecond = new Tuner("Score/scoring_speed_meters_per_sec", 0.45, true);
  static final Tuner scoringChuterShooterSpeed = new Tuner("Score/scoring_chuter_shooter_speed", 0.2, true);

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
        return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS + chuteTofDistanceMeters.get();
      case FRONT_RIGHT:
        return Constants.Field.REEF_CORNER_TO_FAR_POST_METERS + chuteTofDistanceMeters.get();
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
        return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS + chuteTofDistanceMeters.get();
      case FRONT_RIGHT:
        return Constants.Field.REEF_CORNER_TO_NEAR_POST_METERS + chuteTofDistanceMeters.get();
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

  public static Command score(Drive drive, Superstructure superstructure, Constants.ReefBar reefBar, ChuterShooter chuterShooter) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    StringPublisher tofTopic = nt.getStringTopic(Constants.NT.TOF_MODE).publish();
    DoubleArrayTopic tsDistTopic = nt.getDoubleArrayTopic(Constants.NT.TS_DIST_MM);
    FloatArrayTopic cornerTopic = nt.getFloatArrayTopic(Constants.NT.CORNERS);
    ParallelRaceGroup cancellableGroup = new ParallelRaceGroup();
    ScoreCommandState commandState = new ScoreCommandState();
    // FIXME make this not atomic and cringe
    final AtomicReference<Constants.ToFSensorLocation> sensorLocation = new AtomicReference<>(
        Constants.ToFSensorLocation.NONE_SELECTED);

    // We never stop listening to this
    nt.addListener(cornerTopic, EnumSet.of(Kind.kValueAll), networkTableEvent -> {
      float[] results = networkTableEvent.valueData.value.getFloatArray();
      commandState.cornerTimestamp.set(results[0]);
      commandState.cornerDistanceMm.set(results[1]);
      System.out.println("Received results from Pi: " + results[0] + ", " + results[1]);
      commandState.isValid = true;
    });

    nt.addListener(tsDistTopic, EnumSet.of(Kind.kValueAll), networkTableEvent -> {
      double[] results = networkTableEvent.valueData.value.getDoubleArray();
      commandState.distanceTimestamp.set(results[0]);
      commandState.distanceDistanceMm.set(results[1]);
    });

    Command command = Commands.sequence(
        Commands.race(
            Commands.run(() -> {
              drive.setTankDrive(new ChassisSpeeds(scoringSpeedMetersPerSecond.get(), 0, 0));
            }),

            Commands.sequence(
                Commands.runOnce(() -> {
                  System.out.println("Speed before wait: " + drive.getLeftVelocityMetersPerSec());
                  commandState.isValid = false;
                }),
                Commands.race(
                    Commands.waitSeconds(2),
                    Commands.waitUntil(() -> {
                      return Math.abs(drive.getLeftVelocityMetersPerSec() - scoringSpeedMetersPerSecond.get()) < 0.03;
                    })),
                Commands.runOnce(() -> {
                  var speed = drive.getLeftVelocityMetersPerSec();
                  System.out.println("Sending start to Pi with speed: " + speed);
                  tofTopic.set("corner");
                  // FIXME: Check that this left/right is correct
                  if (superstructure.chute.getPivotGoalRads() > 0) {
                    sensorLocation.set(Constants.ToFSensorLocation.FRONT_LEFT);
                  } else {
                    sensorLocation.set(Constants.ToFSensorLocation.FRONT_RIGHT);
                  }
                }),
                Commands.waitUntil(() -> commandState.isValid),
                Commands.runOnce(() -> {
                  Optional<Double> leftEncoderDistance = drive.getLeftPositionAtTime(commandState.cornerTimestamp.get());
                  leftEncoderDistance.ifPresentOrElse(distance -> {
                    commandState.shotCalc = new ShotCalc(distance, commandState.cornerDistanceMm.get(), reefBar, sensorLocation.get());
                  }, () -> cancellableGroup.addCommands(Commands.runOnce(() -> {
                    System.err.println("Failed to find scoring encoder distance because we have no position data");
                  })));
                }))),

        Commands.race(
          new RampDownSpeedCommand(drive, () -> commandState.shotCalc.getTargetPos() - drive.getLeftPositionMeters(), 5.0),
          Commands.run(() -> {
            double distanceTimestamp = commandState.distanceTimestamp.get();
            if (commandState.lastProcessedTimestamp < distanceTimestamp) {
              var optionalLeft = drive.getLeftPositionAtTime(distanceTimestamp);
              optionalLeft.ifPresent((pos) -> {
                commandState.shotCalc.update(pos, commandState.distanceDistanceMm.get());
              });
              commandState.lastProcessedTimestamp = distanceTimestamp;
            }

            // Using latest shot calculation, update chute setpoint constantly
            // to match what the estimates say will be the right value when
            // we reach the shot position.
            superstructure.setChutePivotGoalRads(commandState.shotCalc.getChuteAngleRads());
          })
        ),
        // Corner found, start slowing down to shoot
        Commands.run(() -> {
          System.out.println("Trying to shoot");
          chuterShooter.setShooterMotor(commandState.shotCalc.getShooterSpeed());
        }).withTimeout(2.0),
        Commands.runOnce(() -> {
          System.out.println("Stopping shooting");
          chuterShooter.stopShooting();
        }));
    command.addRequirements(drive, superstructure);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      System.out.println("Command completed: interrupted? " + interrupted);
      tofTopic.set("none");
      // piState.set(new double[] { SensorState.NONE.piValue(),
      // Constants.Drive.SCORING_SPEED });
      commandState.reset();
      chuterShooter.stopShooting();
      // FIXME: Reset? Detect if coral was shot?
    });
  }

  public static Command rampDownSpeed(Drive drive, double targetDistanceMeters, double maxDeceleration) {
    return new RampDownSpeedCommand(drive, () -> targetDistanceMeters, maxDeceleration);
  }
}
