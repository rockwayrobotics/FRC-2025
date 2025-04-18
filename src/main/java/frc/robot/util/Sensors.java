package frc.robot.util;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AsynchronousInterrupt;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class Sensors {
  private DigitalInput grabberAcquiredSensor = new DigitalInput(
      Constants.Digital.ALGAE_DISTANCE_SENSOR);
  private DigitalInput grabberHomeSwitch = new DigitalInput(
      Constants.Digital.ALGAE_HOME_LIMIT_SWITCH);
  private DigitalInput chuteHomeSwitch = new DigitalInput(Constants.Digital.CHUTE_HOME_LIMIT_SWITCH);
  private DigitalInput chuteCoralReadyBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_SHOOT_CORAL_BEAMBREAK);
  private DigitalInput chuteCoralLoadedBeambreak = new DigitalInput(
      Constants.Digital.CHUTE_LOAD_CORAL_BEAMBREAK);
  // private DigitalInput ChuteLoadCoralBeambreak = new DigitalInput(3);
  // private DigitalInput elevatorHomeBeambreak = new DigitalInput(
  //     Constants.Digital.ELEVATOR_HOME_BEAMBREAK);

  public boolean overrideChuteHomeSwitch = false;

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
    Logger.recordOutput("Grabber/acquired", getGrabberAcquired());
    Logger.recordOutput("Grabber/home_sw", getGrabberHomeSwitch());
    Logger.recordOutput("Chute/home_sw", getChuteHomeSwitch());
    Logger.recordOutput("Chute/loaded_sw", getChuteCoralLoadedBeambreak());
    Logger.recordOutput("Chute/ready_sw", getChuteCoralReadyBeambreak());
    // Logger.recordOutput("Elevator/home_sw", getElevatorHomeBeambroken());
  }

  /**
   * @return true if something close to algae grabber
   */
  public boolean getGrabberAcquired() {
    return !grabberAcquiredSensor.get();
  }

  /** @return true if the grabber home limit switch is pressed. */
  public boolean getGrabberHomeSwitch() {
    return !grabberHomeSwitch.get();
  }

  /** @return true if the chute home limit switch is pressed. */
  public boolean getChuteHomeSwitch() {
    if (overrideChuteHomeSwitch) {
      return true;
    }
    return !chuteHomeSwitch.get();
  }

  /** @return true when beam is broken */
  public boolean getChuteCoralReadyBeambreak() {
    return chuteCoralReadyBeambreak.get();
  }

  /** @return true when beam is broken */
  public boolean getChuteCoralLoadedBeambreak() {
    return chuteCoralLoadedBeambreak.get();
  }

  // /** @return true when elevator beam is broken */
  // public boolean getElevatorHomeBeambroken() {
  //   return !elevatorHomeBeambreak.get();
  // }

  private AsynchronousInterrupt registerInterrupt(DigitalInput input,
      TriConsumer<AsynchronousInterrupt, Boolean, Boolean> callback) {
    final AsynchronousInterrupt[] holder = new AsynchronousInterrupt[1];
    final AsynchronousInterrupt interrupt = new AsynchronousInterrupt(input, (rising, falling) -> {
      callback.accept(holder[0], rising, falling);
    });
    holder[0] = interrupt;
    interrupt.setInterruptEdges(true, true);
    interrupt.enable();
    return interrupt;
  }

  // public AsynchronousInterrupt registerElevatorHomeInterrupt(
  //     TriConsumer<AsynchronousInterrupt, Boolean, Boolean> callback) {
  //   return registerInterrupt(elevatorHomeBeambreak, callback);
  // }

  public AsynchronousInterrupt registerChuteHomeInterrupt(
      TriConsumer<AsynchronousInterrupt, Boolean, Boolean> callback) {
    return registerInterrupt(chuteHomeSwitch, callback);
  }

  public AsynchronousInterrupt registerGrabberHomeInterrupt(
      TriConsumer<AsynchronousInterrupt, Boolean, Boolean> callback) {
    return registerInterrupt(grabberHomeSwitch, callback);
  }
}
