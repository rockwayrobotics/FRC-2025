package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Sensors {
  private AnalogInput GrabberAcquiredDistanceSensor = new AnalogInput(
      Constants.Analog.ALGAE_DISTANCE_SENSOR);
  private DigitalInput GrabberHomeSwitch = new DigitalInput(
      Constants.Digital.ALGAE_HOME_LIMIT_SWITCH);
  private DigitalInput ChuteHomeSwitch = new DigitalInput(Constants.Digital.CHUTE_HOME_LIMIT_SWITCH);
  private DigitalInput ChuteCoralReadyBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_SHOOT_CORAL_BEAMBREAK);
  private DigitalInput ChuteCoralLoadedBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_LOAD_CORAL_BEAMBREAK);
  // private DigitalInput ChuteLoadCoralBeambreak = new DigitalInput(3);
  private DigitalInput ElevatorHomeBeambreak = new DigitalInput(
      Constants.Digital.ELEVATOR_HOME_BEAMBREAK);

  private DoublePublisher GrabberAcquiredDistanceSensorPublisher;
  private BooleanPublisher GrabberHomeSwitchPublisher;
  private BooleanPublisher ChuteHomeSwitchPublisher;
  private BooleanPublisher ChuteShootCoralBeambreakPublisher;
  private BooleanPublisher ChuteLoadCoralBeambreakPublisher;
  private BooleanPublisher ElevatorHomeBeambreakPublisher;

  private static Sensors Instance = null;

  private Sensors() {
    var nt = NetworkTableInstance.getDefault();
    this.GrabberAcquiredDistanceSensorPublisher = nt
        .getDoubleTopic("/AdvantageKit/RealOutputs/Grabber/acquired_distance_unknown_units").publish();
    // FIXME: determine units. It also floats around zero when nothing is in range
    this.GrabberHomeSwitchPublisher = nt.getBooleanTopic("/AdvantageKit/RealOutputs/Grabber/home_sw").publish();
    this.ChuteHomeSwitchPublisher = nt.getBooleanTopic("/AdvantageKit/RealOutputs/Chute/home_sw").publish();
    this.ChuteShootCoralBeambreakPublisher = nt.getBooleanTopic("/AdvantageKit/RealOutputs/Chute/ready_sw").publish();
    this.ChuteLoadCoralBeambreakPublisher = nt.getBooleanTopic("/AdvantageKit/RealOutputs/Chute/loaded_sw").publish();
    this.ElevatorHomeBeambreakPublisher = nt.getBooleanTopic("/AdvantageKit/RealOutputs/Elevator/home_sw").publish();

    updateNT();
  }

  public static Sensors getInstance() {
    if (Instance == null) {
      Instance = new Sensors();
    }
    return Instance;
  }

  public void updateNT() {
    this.GrabberAcquiredDistanceSensorPublisher.set(getGrabberAcquiredDistance());
    this.GrabberHomeSwitchPublisher.set(getGrabberHomeSwitch());
    this.ChuteHomeSwitchPublisher.set(getChuteHomeSwitch());
    this.ChuteShootCoralBeambreakPublisher.set(getChuteCoralReadyBeambreak());
    this.ChuteLoadCoralBeambreakPublisher.set(getChuteCoralLoadedBeambreak());
    this.ElevatorHomeBeambreakPublisher.set(getElevatorHomeBeambreak());
  }

  /**
   * @return the distance from the algae acquired distance sensor, in unknown
   *         units, FIXME! see above
   */
  public double getGrabberAcquiredDistance() {
    return GrabberAcquiredDistanceSensor.getVoltage();
  }

  /**
   * @return true if the grabber home limit switch is pressed. It is normally
   *         'true'
   *         when not pressed
   */
  public boolean getGrabberHomeSwitch() {
    return GrabberHomeSwitch.get();
  }

  /**
   * @return true if the chute home limit switch is pressed. It is normally
   *         'true' when not pressed
   */
  public boolean getChuteHomeSwitch() {
    return ChuteHomeSwitch.get();
  }

  /** @return False when beam is broken */
  public boolean getChuteCoralReadyBeambreak() {
    return ChuteCoralReadyBeambreak.get();
  }

  /** @return False when beam is broken */
  public boolean getChuteCoralLoadedBeambreak() {
    return ChuteCoralLoadedBeambreak.get();
  }

  /** @return False when elevator is home */
  public boolean getElevatorHomeBeambreak() {
    return ElevatorHomeBeambreak.get();
  }
}
