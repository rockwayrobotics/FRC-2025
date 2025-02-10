package frc.robot.simulation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose3d;
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

public class WorldSimulation {
  private GyroIO gyro;
  private DriveIOSim driveIO;
  private Drive drive;
  private ChuteIOSim chute;
  private ClimpIOSim climp;
  private ElevatorIOSim elevator;
  private GrabberIOSim grabber;

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

  public void addCoral() {
    var coral = new Coral();
    coral.insertIntoChute(0);
    corals.add(coral);
  }

  public void simulationPeriodic() {
    List<Coral> coralToRemove = new ArrayList<>();
    final List<Pose3d> coralPoses = new ArrayList<>();
    for (Coral coral : corals) {
      coral.periodic(chute);
      if (!coral.isInChute()) {
        coralToRemove.add(coral);
      } else {
        coralPoses.add(coral.getPose(drive.getPose(), elevator, chute));
      }
    }

    corals.removeAll(coralToRemove);
    Logger.recordOutput("FieldSimulation/Corals", coralPoses.toArray(Pose3d[]::new));
  }
}
