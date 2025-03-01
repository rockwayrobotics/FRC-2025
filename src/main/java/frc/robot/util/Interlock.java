package frc.robot.util;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;

/**
 * A utility class for creating tunable number values that can be persisted
 *
 * between robot restarts.
 *
 * The values can be adjusted through NetworkTables and optionally saved in
 * WPILib Preferences.
 */
public class Interlock {
  private final String key = "enable";
  private final BooleanSubscriber subscriber;

  /**
   * defaultValue will be ignored if persist is true, and has been previously set
   */
  public Interlock(String subsystem) {
    var nt = NetworkTableInstance.getDefault();
    var topic = nt.getBooleanTopic(String.join("", "/Test/", subsystem, key));

    this.subscriber = topic.subscribe(false);
    topic.publish().set(false);
  }

  public boolean get() {
    return subscriber.get();
  }

  public NetworkTableListener addListener(Consumer<NetworkTableEvent> callback) {
    return NetworkTableListener.createListener(this.subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), callback);
  }
}
