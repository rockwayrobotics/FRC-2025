package frc.robot.simulation;

import frc.robot.subsystems.chute.ChuteIO;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.climp.ClimpIO;
import frc.robot.subsystems.climp.ClimpIOSim;
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
  private DriveIOSim drive;
  private ChuteIOSim chute;
  private ClimpIOSim climp;
  private ElevatorIOSim elevator;
  private GrabberIOSim grabber;

  public WorldSimulation() {
    this(true);
  }

  public WorldSimulation(boolean simpleDriveSim) {
    if (simpleDriveSim) {
      this.gyro = new GyroIO() {};
      this.drive = new DriveIOSimLite();
    } else {
      var gyro = new GyroIOSim();
      this.gyro = gyro;
      this.drive = new DriveIOSimComplex(gyro);
    }
    this.chute = new ChuteIOSim();
    this.climp = new ClimpIOSim();
    this.elevator = new ElevatorIOSim(0, 0);
    this.grabber = new GrabberIOSim();
  }

  public DriveIO getDrive() {
    return drive;
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

  public void simulationPeriodic() {

  }
}
