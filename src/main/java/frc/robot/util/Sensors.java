package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Sensors {
  private AnalogInput AlgaeAcquiredDistanceSensor = new AnalogInput(
      Constants.Analog.ALGAE_DISTANCE_SENSOR);
  private DigitalInput AlgaeHomeLimitSwitch = new DigitalInput(
      Constants.Digital.ALGAE_HOME_LIMIT_SWITCH);
  private DigitalInput ChuteHomeLimitSwitch = new DigitalInput(Constants.Digital.CHUTE_HOME_LIMIT_SWITCH);
  private DigitalInput ChuteShootCoralBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_SHOOT_CORAL_BEAMBREAK);
  private DigitalInput ChuteLoadCoralBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_LOAD_CORAL_BEAMBREAK);
  // private DigitalInput ChuteLoadCoralBeambreak = new DigitalInput(3);
  private DigitalInput ElevatorHomeBeambreak = new DigitalInput(
      Constants.Digital.ELEVATOR_HOME_BEAMBREAK);

  private DoublePublisher AlgaeAcquiredDistanceSensorPublisher;
  private BooleanPublisher AlgaeHomeLimitSwitchPublisher;
  private BooleanPublisher ChuteHomeLimitSwitchPublisher;
  private BooleanPublisher ChuteShootCoralBeambreakPublisher;
  private BooleanPublisher ChuteLoadCoralBeambreakPublisher;
  private BooleanPublisher ElevatorHomeBeambreakPublisher;

  private static Sensors Instance = null;

  private Sensors() {
    var nt = NetworkTableInstance.getDefault();
    this.AlgaeAcquiredDistanceSensorPublisher = nt
        .getDoubleTopic(String.join("", "/Sensors/", "AlgaeAcquiredDistanceSensor")).publish();
    this.AlgaeHomeLimitSwitchPublisher = nt.getBooleanTopic(String.join("", "/Sensors/", "AlgaeHomeLimitSwitch"))
        .publish();
    this.ChuteHomeLimitSwitchPublisher = nt.getBooleanTopic(String.join("", "/Sensors/", "ChuteHomeLimitSwitch"))
        .publish();
    this.ChuteShootCoralBeambreakPublisher = nt
        .getBooleanTopic(String.join("", "/Sensors/", "ChuteShootCoralBeambreak")).publish();
    this.ChuteLoadCoralBeambreakPublisher = nt.getBooleanTopic(String.join("", "/Sensors/", "ChuteLoadCoralBeambreak"))
        .publish();
    this.ElevatorHomeBeambreakPublisher = nt.getBooleanTopic(String.join("", "/Sensors/", "ElevatorHomeBeambreak"))
        .publish();

    updateNT();
  }

  public static Sensors getInstance() {
    if (Instance == null) {
      Instance = new Sensors();
    }
    return Instance;
  }

  public void updateNT() {
    this.AlgaeAcquiredDistanceSensorPublisher.set(getAlgaeAcquiredDistanceSensor());
    this.AlgaeHomeLimitSwitchPublisher.set(getAlgaeHomeLimitSwitch());
    this.ChuteHomeLimitSwitchPublisher.set(getChuteHomeLimitSwitch());
    this.ChuteShootCoralBeambreakPublisher.set(getChuteShootCoralBeambreak());
    this.ChuteLoadCoralBeambreakPublisher.set(getChuteLoadCoralBeambreak());
    this.ElevatorHomeBeambreakPublisher.set(getElevatorHomeBeambreak());
  }

  public double getAlgaeAcquiredDistanceSensor() {
    return AlgaeAcquiredDistanceSensor.getVoltage();
  }

  public boolean getAlgaeHomeLimitSwitch() {
    return AlgaeHomeLimitSwitch.get();
  }

  public boolean getChuteHomeLimitSwitch() {
    return ChuteHomeLimitSwitch.get();
  }

  public boolean getChuteShootCoralBeambreak() {
    return ChuteShootCoralBeambreak.get();
  }

  public boolean getChuteLoadCoralBeambreak() {
    return ChuteLoadCoralBeambreak.get();
  }

  public boolean getElevatorHomeBeambreak() {
    return ElevatorHomeBeambreak.get();
  }
}
