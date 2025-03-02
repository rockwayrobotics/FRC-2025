package frc.robot.util;

import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Sensors {
  private AnalogInput grabberAcquiredDistanceSensor = new AnalogInput(
      Constants.Analog.ALGAE_DISTANCE_SENSOR);
  private DigitalInput grabberHomeSwitch = new DigitalInput(
      Constants.Digital.ALGAE_HOME_LIMIT_SWITCH);
  private DigitalInput chuteHomeSwitch = new DigitalInput(Constants.Digital.CHUTE_HOME_LIMIT_SWITCH);
  private DigitalInput chuteCoralReadyBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_SHOOT_CORAL_BEAMBREAK);
  private DigitalInput chuteCoralLoadedBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_LOAD_CORAL_BEAMBREAK);
  // private DigitalInput ChuteLoadCoralBeambreak = new DigitalInput(3);
  private DigitalInput elevatorHomeBeambreak = new DigitalInput(
      Constants.Digital.ELEVATOR_HOME_BEAMBREAK);

  private static Sensors instance = null;

  private Sensors() {
    updateNT();
  }

  public static Sensors getInstance() {
    if (instance == null) {
      instance = new Sensors();
    }
    return instance;
  }

  public void updateNT() {
    Logger.recordOutput("Grabber/distance_mm", getGrabberAcquiredDistance());
    Logger.recordOutput("Grabber/home_sw", getGrabberHomeSwitch());
    Logger.recordOutput("Chute/home_sw", getChuteHomeSwitch());
    Logger.recordOutput("Chute/loaded_sw", getChuteCoralLoadedBeambreak());
    Logger.recordOutput("Chute/ready_sw", getChuteCoralReadyBeambreak());
    Logger.recordOutput("Elevator/home_sw", getElevatorHomeBeambroken());
  }

  /**
   * @return the distance from the algae acquired distance sensor, in mm (FIXME:
   *         rn its in volts)
   */
  public double getGrabberAcquiredDistance() {
    // TODO: Calibrate this and convert from voltage to mm.
    // 40 to 300 mm
    return grabberAcquiredDistanceSensor.getVoltage();
  }

  /** @return true if the grabber home limit switch is pressed. */
  public boolean getGrabberHomeSwitch() {
    return grabberHomeSwitch.get();
  }

  /** @return true if the chute home limit switch is pressed. */
  public boolean getChuteHomeSwitch() {
    return chuteHomeSwitch.get();
  }

  /** @return false when beam is broken */
  public boolean getChuteCoralReadyBeambreak() {
    return chuteCoralReadyBeambreak.get();
  }

  /** @return false when beam is broken */
  public boolean getChuteCoralLoadedBeambreak() {
    return chuteCoralLoadedBeambreak.get();
  }

  /** @return true when elevator beam is broken */
  public boolean getElevatorHomeBeambroken() {
    return !elevatorHomeBeambreak.get();
  }

  private AsynchronousInterrupt registerInterrupt(DigitalInput input, boolean risingEdge, Consumer<Boolean> callback) {
    var interrupt = new AsynchronousInterrupt(input, (rising, falling) -> {
      callback.accept(risingEdge ? rising : falling);
    });
    interrupt.setInterruptEdges(risingEdge, !risingEdge);
    interrupt.enable();
    return interrupt;
  }

  public AsynchronousInterrupt registerElevatorHomeInterrupt(boolean risingEdge, Consumer<Boolean> callback) {
    return registerInterrupt(elevatorHomeBeambreak, risingEdge, callback);
  }

  public AsynchronousInterrupt registerChuteHomeInterrupt(boolean risingEdge, Consumer<Boolean> callback) {
    return registerInterrupt(chuteHomeSwitch, risingEdge, callback);
  }

  public AsynchronousInterrupt registerGrabberHomeInterrupt(boolean risingEdge, Consumer<Boolean> callback) {
    return registerInterrupt(grabberHomeSwitch, risingEdge, callback);
  }
}
