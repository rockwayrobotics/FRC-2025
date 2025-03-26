package frc.robot.simulation;

import java.util.EnumSet;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArrayPublisher;
import edu.wpi.first.networktables.FloatPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.simulation.WorldSimulation.FieldObstacles;

public class PiSimulation {
  public static final String DEFAULT_STATE = "none";
  private final StringSubscriber chuteModeSubscriber;
  private final StringSubscriber tofModeSubscriber;
  private final FloatArrayPublisher cornerPublisher;
  private final DoubleArrayPublisher tsDistPublisher;
  private final FieldObstacles fieldObstacles;
  private String piMode = DEFAULT_STATE;

  private NetworkTableListenerPoller modePoller;
  private CornerDetector cornerDetector;
  private boolean sawCorner = false;
  private String tofMode = "none";

  public record TimeDistance(float time, float distance) {
  }

  public record CornerInfo(double cornerTimestamp, double angle) {
  }

  public PiSimulation(FieldObstacles fieldObstacles) {
    NetworkTableInstance nt = NetworkTableInstance.getDefault();
    chuteModeSubscriber = nt.getStringTopic(Constants.NT.CHUTE_MODE).subscribe("none");
    tofModeSubscriber = nt.getStringTopic(Constants.NT.TOF_MODE).subscribe("none");
    modePoller = new NetworkTableListenerPoller(NetworkTableInstance.getDefault());
    modePoller.addListener(chuteModeSubscriber, EnumSet.of(Kind.kValueAll));
    modePoller.addListener(tofModeSubscriber, EnumSet.of(Kind.kValueAll));
    cornerPublisher = nt.getFloatArrayTopic(Constants.NT.CORNERS).publish();
    tsDistPublisher = nt.getDoubleArrayTopic(Constants.NT.TS_DIST_MM).publish();
    this.fieldObstacles = fieldObstacles;
  }

  private void simulateReading(Pose2d robotPose, String chuteSide) {
    if (piMode.equals("none")) {
      return;
    }

    Transform2d sensorTransform = null;
    if (chuteSide.equals("right")) {
      sensorTransform = Constants.ToFSensor.FRONT_RIGHT;
    } else if (chuteSide.equals("left")) {
      sensorTransform = Constants.ToFSensor.FRONT_LEFT;
    }

    if (sensorTransform == null) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    Translation2d sensorPose = robotPose
        .transformBy(new Transform2d(sensorTransform.getTranslation(), robotPose.getRotation()))
        .getTranslation();
    double angle = robotPose.getRotation().getRadians()
        + sensorTransform.getRotation().getRadians();
    double distanceMm = ToFSimUtils.simulateSensor(sensorPose, angle, fieldObstacles);
    if (distanceMm < Double.POSITIVE_INFINITY) {
      cornerDetector.addRecord(now, (float) distanceMm);
      if (cornerDetector.foundCorner()) {
        sawCorner = true;
        // TODO: possibly adjust time?
        var corner_ts = cornerDetector.getCornerTimestamp();
        cornerPublisher.set(new float[] {
          (float) corner_ts, (float) cornerDetector.getCornerDistance()
        });
        cornerDetector.reset();
      }

      if (tofMode.equals("corner")) {
        // TODO: possibly adjust time?
        tsDistPublisher.set(new double[] {
          now, distanceMm
        });

        // Use sawCorner to decide if we should flush?
      }
      System.out.println("Distance: " + distanceMm + "mm");
    }
  }

  public void periodic(Pose2d robotPose) {
    String chuteSide = "right";
    NetworkTableEvent[] events = modePoller.readQueue();
    for (NetworkTableEvent event : events) {
      var topic = event.valueData.getTopic().getName();
      if (topic.equals(Constants.NT.CHUTE_MODE)) {
        var value = event.valueData.value.getString();
        System.out.println("chute: " + value);
        // pos, _, side
        String[] pos_side = value.split("/");
        String side = pos_side[2];
        if (!pos_side[0].equals("load")) {
          if (side.equals("right")) {
            side = "left";
          } else {
            side = "right";
          }
        }
        if (!chuteSide.equals(side)) {
          chuteSide = side;
          cornerDetector.reset();
        }
      } else if (topic.equals(Constants.NT.TOF_MODE)) {
        tsDistPublisher.set(new double[] {0, 0});
        tofMode = event.valueData.value.getString();
        sawCorner = false;
        System.out.println("tof mode: " + tofMode);
      }
    }

    simulateReading(robotPose, chuteSide);
  }
}
