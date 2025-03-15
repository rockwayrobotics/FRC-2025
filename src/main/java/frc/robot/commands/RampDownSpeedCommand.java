package frc.robot.commands;

import java.util.EnumSet;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RampDownSpeed;

public class RampDownSpeedCommand extends Command {
  static class TsDist {
    public double timestamp;
    public double distanceMm;
    public TsDist(double[] results) {
      timestamp = results[0];
      distanceMm = results[1];
    }
  }

  Drive drive;
  DoubleSupplier targetDistanceMetersSupplier;
  double maxDeceleration;
  double initialPositionMeters;
  AtomicReference<TsDist> firstMeasurement = new AtomicReference<>();
  AtomicReference<TsDist> currentMeasurement = new AtomicReference<>();
  double angleFromWallRadians = 0;
  RampDownSpeed ramp;
  double parallelDistance;

  public RampDownSpeedCommand(Drive drive, DoubleSupplier targetDistanceMetersSupplier, double maxDeceleration) {
    this.drive = drive;
    this.targetDistanceMetersSupplier = targetDistanceMetersSupplier;
    this.maxDeceleration = maxDeceleration;
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    DoubleArrayTopic tsDistTopic = nt.getDoubleArrayTopic(Constants.NT.TS_DIST_MM);
    nt.addListener(tsDistTopic, EnumSet.of(Kind.kValueAll), networkTableEvent -> {
      double[] results = networkTableEvent.valueData.value.getDoubleArray();
      if (firstMeasurement.get() == null) {
        firstMeasurement.set(new TsDist(results));
      } else {
        currentMeasurement.set(new TsDist(results));
      }
    });

    this.addRequirements(drive);
  }

  private double getPosition() {
    return (drive.getLeftPositionMeters() + drive.getRightPositionMeters()) / 2;
  }

  private double getVelocity() {
    return (drive.getLeftVelocityMetersPerSec() + drive.getRightVelocityMetersPerSec()) / 2;
  }

  private double remainingDistance() {
    return Math.cos(angleFromWallRadians) * parallelDistance - (getPosition() - initialPositionMeters);
  }

  @Override
  public void initialize() {
    angleFromWallRadians = 0;
    firstMeasurement.set(null);
    currentMeasurement.set(null);
    parallelDistance = targetDistanceMetersSupplier.getAsDouble();
    this.ramp = new RampDownSpeed(getVelocity(), targetDistanceMetersSupplier.getAsDouble(), maxDeceleration);
    this.initialPositionMeters = getPosition();
  }

  @Override
  public void execute() {
    TsDist current = currentMeasurement.get();
    if (current != null) {
      TsDist first = firstMeasurement.get();
      var optFirstPos = drive.getLeftPositionAtTime(first.timestamp);
      var optCurrentPos = drive.getLeftPositionAtTime(current.timestamp);
      if (optFirstPos.isPresent() && optCurrentPos.isPresent()) {
        double distanceTravelledMeters = optCurrentPos.get() - optFirstPos.get();
        double deltaReefDistanceMm = current.distanceMm - first.distanceMm;
        angleFromWallRadians = Math.atan2(deltaReefDistanceMm, distanceTravelledMeters * 1000);
        Logger.recordOutput("Scoring/RampDown/angle_deg", Units.radiansToDegrees(angleFromWallRadians));

        // The offset from the edge of the wall to the end of the post (perpendicular)
        // may be useful?
        double wallToPostOffsetMm = 53.6; // Measured from field CAD

      }
    }
    var speed = ramp.calculateSpeed(remainingDistance());
    drive.setTankDrive(new ChassisSpeeds(speed, 0, 0));
  }

  @Override
  public boolean isFinished() {
    return remainingDistance() < 0.05;
  }

  @Override
  public void end(boolean cancelled) {
    drive.stop();
  }
}
