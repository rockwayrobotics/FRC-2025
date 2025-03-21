package frc.robot.commands;

import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.FloatArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.util.ShotCalc;
import frc.robot.util.Tuner;
import frc.robot.Constants;
import frc.robot.Constants.CoralLevel;
import frc.robot.subsystems.chuterShooter.ChuterShooter;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.Superstructure;

public class ScoreCommandsOnlyDrive {
  public static class ScoreCommandState {
    public double requestCornerTimestamp = 0;
    public AtomicReference<Float> cornerTimestamp = new AtomicReference<>();
    public AtomicReference<Float> cornerDistanceMm = new AtomicReference<>();
    public AtomicReference<Double> distanceTimestamp = new AtomicReference<>();
    public AtomicReference<Double> distanceDistanceMm = new AtomicReference<>();
    public double lastProcessedTimestamp;
    public boolean isValid;
    public ShotCalc shotCalc;

    public ScoreCommandState() {
      reset();
    }

    public void reset() {
      requestCornerTimestamp = 0;
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
  static final Tuner earlyShotMillimeterTuner = new Tuner("Score/early_shot_mm", 120, true);
  static final Tuner angleProportionalTuner = new Tuner("Score/angle_proportional_early_shot", 0, true);
  static final Tuner speedProportionalTuner = new Tuner("Score/speed_proportional_early_shot", 0, true);

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

  public static Command score(Drive drive, Superstructure superstructure, Constants.ReefBar reefBar,
      ChuterShooter chuterShooter) {
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
                      return Math.abs(drive.getLeftVelocityMetersPerSec() - scoringSpeedMetersPerSecond.get()) < 0.1;
                    })),
                Commands.runOnce(() -> {
                  var speed = drive.getLeftVelocityMetersPerSec();
                  System.out.println("Sending start to Pi with speed: " + speed);
                  commandState.requestCornerTimestamp = Timer.getFPGATimestamp();
                  tofTopic.set("corner");
                  // FIXME: Check that this left/right is correct
                  if (superstructure.chute.getPivotGoalRads() > 0) {
                    sensorLocation.set(Constants.ToFSensorLocation.FRONT_LEFT);
                  } else {
                    sensorLocation.set(Constants.ToFSensorLocation.FRONT_RIGHT);
                  }
                }),
                Commands.waitUntil(() -> commandState.isValid && commandState.cornerTimestamp.get() > commandState.requestCornerTimestamp),
                Commands.runOnce(() -> {
                  Optional<Double> leftEncoderDistance = drive
                      .getLeftPositionAtTime(commandState.cornerTimestamp.get());
                  Logger.recordOutput("/ScoreCommands/corner_timestamp", commandState.cornerTimestamp.get());
                  leftEncoderDistance.ifPresentOrElse(distance -> {
                    Logger.recordOutput("/ScoreCommands/distance", distance);
                    Logger.recordOutput("/ScoreCommands/corner_distance", commandState.cornerDistanceMm.get());
                    commandState.shotCalc = new ShotCalc(distance * 1000.0, commandState.cornerDistanceMm.get(),
                        reefBar, sensorLocation.get());
                  }, () -> cancellableGroup.addCommands(Commands.runOnce(() -> {
                    System.err.println("Failed to find scoring encoder distance because we have no position data");
                  })));
                }))),

        // Corner found, keep driving and now use the shot calculator for
        // distance/angle.
        Commands.race(
            Commands.run(() -> {
              drive.setTankDrive(new ChassisSpeeds(scoringSpeedMetersPerSecond.get(), 0, 0));
            }).withTimeout(2),
            Commands.waitUntil(() -> {
              if (commandState.shotCalc.getTargetDist() <= 0.0) {
                // No target distance yet means we haven't received any readings, wait for at
                // least one.
                return false;
              }

              double currentEncoderMm = drive.getLeftPositionMeters() * 1000;
              double millimetersPerPeriodic = drive.getLeftVelocityMetersPerSec() * 1000 * 0.02;
              double targetPosMm = commandState.shotCalc.getTargetPos() - earlyShotMillimeterTuner.get()
                  - angleProportionalTuner.get() * commandState.shotCalc.getWallAngleRads()
                  - speedProportionalTuner.get() * scoringSpeedMetersPerSecond.get();
              double nextEncoderMm = currentEncoderMm + millimetersPerPeriodic;

              // If the delayed shot isn't working and tuning is hard, the original condition
              // was:
              // return currentEncoderMm >= targetPosMm;

              if (nextEncoderMm < targetPosMm) {
                // We won't reach the target next periodic, keep going.
                return false;
              }

              double fractionOfPeriod = (targetPosMm - currentEncoderMm) / millimetersPerPeriodic;
              Logger.recordOutput("/ScoreCommands/fraction_of_period", fractionOfPeriod);
              // If we return true and do not schedule a shot, we will start shooting in 20
              // ms, when we expect to be at nextEncoderMm.
              if (fractionOfPeriod < 1.0) {
                chuterShooter.scheduleShoot(commandState.shotCalc.getShooterSpeed(), fractionOfPeriod * 0.02);
              }
              return true;
            }),
            Commands.run(() -> {
              double distanceTimestamp = commandState.distanceTimestamp.get();
              if (commandState.lastProcessedTimestamp < distanceTimestamp) {
                var optionalLeft = drive.getLeftPositionAtTime(distanceTimestamp);
                optionalLeft.ifPresent((pos) -> {
                  commandState.shotCalc.update(pos * 1000, commandState.distanceDistanceMm.get());
                });
                commandState.lastProcessedTimestamp = distanceTimestamp;
              }

              // Using latest shot calculation, update chute setpoint constantly
              // to match what the estimates say will be the right value when
              // we reach the shot position.
              superstructure.setChutePivotGoalRads(commandState.shotCalc.getChuteAngleRads());
            })),

        // Time to shoot, keep driving while we shoot, but slow down.
        Commands.race(
            Commands.sequence(
                Commands.run(() -> {
                  drive.setTankDrive(new ChassisSpeeds(scoringSpeedMetersPerSecond.get(), 0, 0));
                }).withTimeout(0.3),
                new RampDownSpeedCommand(drive,
                    () -> commandState.shotCalc.getTargetPos() / 1000 - drive.getLeftPositionMeters() + 1, 5.0)),
            Commands.sequence(
                Commands.run(() -> {
                  chuterShooter.setShooterMotor(commandState.shotCalc.getShooterSpeed());
                }).withTimeout(2.0),
                Commands.runOnce(() -> {
                  chuterShooter.stopShooting();
                }))));
    command.addRequirements(drive, superstructure, chuterShooter);
    cancellableGroup.addCommands(command);
    return cancellableGroup.finallyDo(interrupted -> {
      System.out.println("Command completed: interrupted? " + interrupted);
      tofTopic.set("none");
      // piState.set(new double[] { SensorState.NONE.piValue(),
      // Constants.Drive.SCORING_SPEED });
      commandState.reset();
      chuterShooter.stopShooting();
      // We wanted to go to safe chute height, but in the rare case that a coral manages to land
      // vertically on the robot, that would be unsafe.
      // Instead, go down to L2 height.
      // Operator doesn't like moving after shot, so L2 height commented out. 
      // superstructure.gotoElevatorL2();

      // FIXME: Reset? Detect if coral was shot?
    });
  }

  public static Command rampDownSpeed(Drive drive, double targetDistanceMeters, double maxDeceleration) {
    return new RampDownSpeedCommand(drive, () -> targetDistanceMeters, maxDeceleration);
  }
}
