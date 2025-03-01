package frc.robot.util;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Sensors {
  private AnalogInput AlgaeAcquiredDistanceSensor = new AnalogInput(
      Constants.Digital.ALGAE_ACQUIRED_DISTANCE_SENSOR);
  private DigitalInput AlgaeHomeLimitSwitch = new DigitalInput(
      Constants.Digital.ALGAE_HOME_LIMIT_SWITCH);
  private DigitalInput ChuteHomeLimitSwitch = new DigitalInput(Constants.Digital.CHUTE_HOME_LIMIT_SWITCH);
  private DigitalInput ChuteShootCoralBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_SHOOT_CORAL_BEAMBREAK);
  private DigitalInput ChuteLoadCoralBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_LOAD_CORAL_BEAMBREAK);
  private DigitalInput ElevatorHomeBeambreak = new DigitalInput(
      Constants.Digital.ELEVATOR_HOME_BEAMBREAK);

  public Sensors() {
    updateNT();
  }

  public void updateNT() {
    var nt = NetworkTableInstance.getDefault();
    nt.getDoubleTopic(String.join("", "/Sensors/", "AlgaeAcquiredDistanceSensor")).publish()
        .set(AlgaeAcquiredDistanceSensor.getVoltage());
    nt.getBooleanTopic(String.join("", "/Sensors/", "AlgaeHomeLimitSwitch")).publish()
        .set(AlgaeHomeLimitSwitch.get());
    nt.getBooleanTopic(String.join("", "/Sensors/", "ChuteHomeLimitSwitch")).publish()
        .set(ChuteHomeLimitSwitch.get());
    nt.getBooleanTopic(String.join("", "/Sensors/", "ChuteShootCoralBeambreak")).publish()
        .set(ChuteShootCoralBeambreak.get());
    nt.getBooleanTopic(String.join("", "/Sensors/", "ChuteLoadCoralBeambreak")).publish()
        .set(ChuteLoadCoralBeambreak.get());
    nt.getBooleanTopic(String.join("", "/Sensors/", "ElevatorHomeBeambreak")).publish()
        .set(ElevatorHomeBeambreak.get());
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
