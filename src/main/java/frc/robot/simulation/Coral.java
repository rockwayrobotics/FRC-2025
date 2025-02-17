package frc.robot.simulation;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

import org.ironmaple.simulation.seasonspecific.reefscape2025.ReefscapeCoralOnFly;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.Constants;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSim;

public class Coral {
  private Pose3d pose = new Pose3d();
  private boolean inChute = false;
  // Position in meters of the lower end of the coral measured with 0 as the
  // "loading" side of the chute.
  private double chutePosition = 0.0;
  private double chuteVelocity = 0.0;
  private ReefscapeCoralOnFly flyingCoral = null;

  public static final double CORAL_MASS_KG = Kilograms.convertFrom(1.4, Pounds);
  public static final double CORAL_LENGTH_METERS = Meters.convertFrom(11.875, Inches);
  // Just a rough guess
  public static final double CORAL_COEFF_OF_FRICTION = 0.3;
  private static final double GRAVITY = 9.81;

  public Coral() {
  }

  public void insertIntoChute(double startVelocity) {
    inChute = true;
    chutePosition = 0.0;
    chuteVelocity = startVelocity;
  }

  public boolean isInChute() {
    return inChute;
  }

  private Pose3d computeInChutePose(Pose2d robotPose, ElevatorIOSim elevator, ChuteIOSim chuteIOSim) {
    return new Pose3d(robotPose)
        // Move the coral up by the height of the pivot
        .plus(new Transform3d(0, 0, Constants.Elevator.MIN_PIVOT_HEIGHT_METERS + elevator.getChutePivotHeightMeters(),
            new Rotation3d()))
        // Rotate it 90 degrees around the z-axis
        .plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, Math.PI / 2)))
        // Rotate it to match the pitch of the chute
        .plus(new Transform3d(0, 0, 0, new Rotation3d(0, -chuteIOSim.getPivotAngleRads(), 0)))
        // Move it along the chute
        .plus(new Transform3d(Constants.Chute.CHUTE_LENGTH_METERS / 2 - chutePosition, 0, 0, new Rotation3d()));
  }

  public Pose3d getPose() {
    return pose;
  }

  public ReefscapeCoralOnFly getFlyingCoral() {
    return flyingCoral;
  }

  public void periodic(Pose2d robotPose, ElevatorIOSim elevator, ChuteIOSim chuteIOSim) {
    this.periodic(0.02, robotPose, elevator, chuteIOSim);
  }

  public void periodic(double dt, Pose2d robotPose, ElevatorIOSim elevator, ChuteIOSim chuteIOSim) {
    if (!inChute) {
      // FIXME: Simulate states outside of chute
      return;
    }

    double chuteAngleRads = chuteIOSim.getPivotAngleRads();
    if (chutePosition < 0) {
      System.out.println("Coral fell out of chute");
      inChute = false;
      chuteIOSim.setCoralLoaded(false);
      // FIXME: Robot is not stopped
      // FIXME: Chute is not always pointed clockwise
      var currentPose = computeInChutePose(robotPose, elevator, chuteIOSim);
      flyingCoral = new ReefscapeCoralOnFly(
          robotPose.getTranslation(),
          new Translation2d(),
          new ChassisSpeeds(0, 0, 0),
          robotPose.getRotation().rotateBy(Rotation2d.kCW_90deg),
          currentPose.getMeasureZ(),
          LinearVelocity.ofBaseUnits(chuteVelocity, MetersPerSecond),
          Angle.ofBaseUnits(chuteIOSim.getPivotAngleRads(), Radians));
    } else if (chutePosition < Constants.Chute.CHUTE_WHEEL_POSITION_METERS) {
      double normalForce = CORAL_MASS_KG * GRAVITY * Math.cos(chuteAngleRads);
      double gravityForce = CORAL_MASS_KG * GRAVITY * Math.sin(chuteAngleRads);

      double frictionForce;
      if (Math.abs(chuteVelocity) < 1e-6) {
        // If not really moving, friction opposes gravity
        frictionForce = Math.min(Math.abs(gravityForce), CORAL_COEFF_OF_FRICTION * normalForce)
            * Math.signum(gravityForce);
      } else {
        // If moving, friction opposes velocity
        frictionForce = CORAL_COEFF_OF_FRICTION * normalForce * Math.signum(chuteVelocity);
      }
      double netForce = gravityForce - frictionForce;

      double acceleration = netForce / CORAL_MASS_KG;
      chuteVelocity += acceleration * dt;
      chutePosition += chuteVelocity * dt;

      if (chutePosition >= Constants.Chute.CHUTE_WHEEL_POSITION_METERS) {
        chutePosition = Constants.Chute.CHUTE_WHEEL_POSITION_METERS;
        chuteIOSim.setCoralLoaded(true);
      }
    } else {
      chuteVelocity = chuteIOSim.getShooterVelocityRadPerSec() * Constants.Chute.SHOOTER_WHEEL_RADIUS_METERS;
      chutePosition += chuteVelocity * dt;

      if (chutePosition >= Constants.Chute.CHUTE_LENGTH_METERS) {
        inChute = false;
        chuteIOSim.setCoralLoaded(false);
        var currentPose = computeInChutePose(robotPose, elevator, chuteIOSim);
        flyingCoral = new ReefscapeCoralOnFly(
            robotPose.getTranslation(),
            // new Translation2d(currentPose.getX() - robotPose.getX(), currentPose.getY() - robotPose.getY()),
            new Translation2d(robotPose.getX() - currentPose.getX(), robotPose.getY() - currentPose.getY()),
            new ChassisSpeeds(0, 0, 0),
            robotPose.getRotation()
                .rotateBy(chuteIOSim.getPivotAngleRads() < Math.PI / 2 ? Rotation2d.kCW_90deg : Rotation2d.kCCW_90deg),
            currentPose.getMeasureZ(),
            LinearVelocity.ofBaseUnits(chuteVelocity * 10, MetersPerSecond),
            Angle.ofBaseUnits(chuteIOSim.getPivotAngleRads(), Radians));

      }
    }
    pose = computeInChutePose(robotPose, elevator, chuteIOSim);
  }
}
