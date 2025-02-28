package frc.robot.util;

import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListener;
import edu.wpi.first.wpilibj.Preferences;

/**
 * A utility class for creating tunable number values that can be persisted
 *
 * between robot restarts.
 *
 * The values can be adjusted through NetworkTables and optionally saved in
 * WPILib Preferences.
 */
public class Tuner {
  private final String key;
  private final double defaultValue;
  private final DoubleSubscriber subscriber;

  /**
   * defaultValue will be ignored if persist is true, and has been previously set
   */
  public Tuner(String name, double defaultValue, boolean persist) {
    this.key = name;
    var nt = NetworkTableInstance.getDefault();
    var topic = nt.getDoubleTopic(String.join("", "/Tuning/", name));
    if (persist && Preferences.containsKey(key)) {
      this.defaultValue = Preferences.getDouble(key, defaultValue);
    } else {
      this.defaultValue = defaultValue;
    }

    this.subscriber = topic.subscribe(this.defaultValue);
    topic.publish().set(this.defaultValue);

    if (persist) {
      Preferences.setDouble(key, this.defaultValue);
      NetworkTableListener.createListener(this.subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll),
          (event) -> {
            Preferences.setDouble(key, this.subscriber.get());
          });
    }
  }

  public double get() {
    return subscriber.get();
  }

  public NetworkTableListener addListener(Consumer<NetworkTableEvent> callback) {
    return NetworkTableListener.createListener(this.subscriber, EnumSet.of(NetworkTableEvent.Kind.kValueAll), callback);
  }
}
