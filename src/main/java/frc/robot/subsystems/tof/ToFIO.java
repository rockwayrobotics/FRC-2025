package frc.robot.subsystems.tof;

import org.littletonrobotics.junction.AutoLog;

public interface ToFIO {
  @AutoLog
  public static class ToFIOInputs {
    // All of these are two-element arrays, [distanceInMm, timestamp]
    public double[] frontLeft = new double[] {0, 0};
    public double[] backLeft = new double[] {0, 0};
    public double[] frontRight = new double[] {0, 0};
    public double[] backRight = new double[] {0, 0};
  }

  public default void updateInputs(ToFIOInputs inputs) {}
}
