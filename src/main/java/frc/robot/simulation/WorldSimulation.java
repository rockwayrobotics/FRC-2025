package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotTracker;
import frc.robot.subsystems.chute.ChuteIO;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.climp.ClimpIO;
import frc.robot.subsystems.climp.ClimpIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSimComplex;
import frc.robot.subsystems.drive.DriveIOSimLite;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.grabber.GrabberIO;
import frc.robot.subsystems.grabber.GrabberIOSim;
import frc.robot.subsystems.tof.ToFIO;
import frc.robot.subsystems.tof.ToFIOSim;

public class WorldSimulation {
  public static final class FieldObstacles {
    public final double fieldWidthMeters = 17.548;
    public final Translation2d[] blueReefVertices;
    public final Translation2d[] redReefVertices;

    public FieldObstacles() {
      blueReefVertices = new Translation2d[] {
          new Translation2d(3.658, 3.546),
          new Translation2d(3.658, 4.506),
          new Translation2d(4.489, 4.987),
          new Translation2d(5.3213, 4.506),
          new Translation2d(5.3213, 3.546),
          new Translation2d(4.489, 3.065),
      };

      redReefVertices = Arrays.stream(blueReefVertices)
          .map(blueVertex -> new Translation2d(fieldWidthMeters - blueVertex.getX(), blueVertex.getY()))
          .toArray(Translation2d[]::new);
    }
  }

  private FieldObstacles fieldObstacles = new FieldObstacles();

  private GyroIO gyro;
  private DriveIOSim driveIO;
  private Drive drive;
  private ChuteIOSim chute;
  private ClimpIOSim climp;
  private ElevatorIOSim elevator;
  private GrabberIOSim grabber;
  private ToFIOSim tof;

  private ArrayList<Coral> corals = new ArrayList<>();

  public WorldSimulation() {
    this(true);
  }

  public WorldSimulation(boolean simpleDriveSim) {
    if (simpleDriveSim) {
      this.gyro = new GyroIO() {
      };
      this.driveIO = new DriveIOSimLite();
    } else {
      var gyro = new GyroIOSim();
      this.gyro = gyro;
      this.driveIO = new DriveIOSimComplex(gyro);
    }
    this.drive = new Drive(driveIO, gyro);
    this.chute = new ChuteIOSim();
    this.climp = new ClimpIOSim();
    this.elevator = new ElevatorIOSim(0);
    this.grabber = new GrabberIOSim();
    this.tof = new ToFIOSim();
  }

  public Drive getDrive() {
    return drive;
  }

  public DriveIO getDriveIO() {
    return driveIO;
  }

  public GyroIO getGyro() {
    return gyro;
  }

  public ChuteIO getChute() {
    return chute;
  }

  public ClimpIO getClimp() {
    return climp;
  }

  public ElevatorIO getElevator() {
    return elevator;
  }

  public GrabberIO getGrabber() {
    return grabber;
  }

  public ToFIO getToF() {
    return tof;
  }

  public void addCoral() {
    var coral = new Coral();
    coral.insertIntoChute(0);
    corals.add(coral);
  }

  private void simulateToF(Pose2d robotPose) {
    Translation2d frontLeftSensor = robotPose
        .transformBy(new Transform2d(Constants.ToFSensor.FRONT_LEFT.getTranslation(), robotPose.getRotation()))
        .getTranslation();
    double frontLeftAngle = robotPose.getRotation().getRadians() + Constants.ToFSensor.FRONT_LEFT.getRotation().getRadians();

    double distance = ToFSimUtils.simulateSensor(frontLeftSensor, frontLeftAngle, fieldObstacles);
    if (distance < Double.POSITIVE_INFINITY) {
      tof.simulateFrontLeft(distance);
    }

    Translation2d backLeftSensor = robotPose
        .transformBy(new Transform2d(Constants.ToFSensor.BACK_LEFT.getTranslation(), robotPose.getRotation()))
        .getTranslation();
    double backLeftAngle = robotPose.getRotation().getRadians() + Constants.ToFSensor.BACK_LEFT.getRotation().getRadians();

    distance = ToFSimUtils.simulateSensor(backLeftSensor, backLeftAngle, fieldObstacles);
    if (distance < Double.POSITIVE_INFINITY) {
      tof.simulateBackLeft(distance);
    }

    Translation2d frontRightSensor = robotPose
        .transformBy(new Transform2d(Constants.ToFSensor.FRONT_RIGHT.getTranslation(), robotPose.getRotation()))
        .getTranslation();
    double frontRightAngle = robotPose.getRotation().getRadians() + Constants.ToFSensor.FRONT_RIGHT.getRotation().getRadians();

    distance = ToFSimUtils.simulateSensor(frontRightSensor, frontRightAngle, fieldObstacles);
    if (distance < Double.POSITIVE_INFINITY) {
      tof.simulateFrontRight(distance);
    }

    Translation2d backRightSensor = robotPose
        .transformBy(new Transform2d(Constants.ToFSensor.BACK_RIGHT.getTranslation(), robotPose.getRotation()))
        .getTranslation();
    double backRightAngle = robotPose.getRotation().getRadians() + Constants.ToFSensor.BACK_RIGHT.getRotation().getRadians();

    distance = ToFSimUtils.simulateSensor(backRightSensor, backRightAngle, fieldObstacles);
    if (distance < Double.POSITIVE_INFINITY) {
      tof.simulateBackRight(distance);
    }
  }

  public void simulationPeriodic() {
    // FIXME: Simulation should really simulate where the robot is, and not use
    // odometry for it
    // so we can simulate errors.
    Pose2d robotPose = RobotTracker.getInstance().getEstimatedPose();
    simulateToF(robotPose);

    List<Coral> coralToRemove = new ArrayList<>();
    final List<Pose3d> coralPoses = new ArrayList<>();
    for (Coral coral : corals) {
      coral.periodic(chute);
      if (!coral.isInChute()) {
        coralToRemove.add(coral);
      } else {
        coralPoses.add(coral.getPose(robotPose, elevator, chute));
      }
    }

    // new Pose3d(0, 0, elevator.getChutePivotHeightMeters(), new
    // Rotation3d(chute.getPivotAngleRads(), 0, 0)),
    // Offset for chute origin is 0.236 up in z.
    Pose3d chutePose = new Pose3d(0, 0, elevator.getChutePivotHeightMeters(), new Rotation3d())
        .transformBy(new Transform3d(0, 0, 0.236, new Rotation3d()))
        .transformBy(new Transform3d(0, 0, 0, new Rotation3d(chute.getPivotAngleRads(), 0, 0)))
        .transformBy(new Transform3d(0, 0, -0.236, new Rotation3d()));
    Logger.recordOutput("Robot/Components", new Pose3d[] {
        // Elevator box
        new Pose3d(0, 0, elevator.getChutePivotHeightMeters(), new Rotation3d(0, 0, 0)),
        // Chute
        chutePose,
        // Climber arm
        new Pose3d(0, 0, 0, new Rotation3d()),
    });

    corals.removeAll(coralToRemove);
    Logger.recordOutput("FieldSimulation/Corals", coralPoses.toArray(Pose3d[]::new));
  }
}
