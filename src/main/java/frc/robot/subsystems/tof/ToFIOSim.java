package frc.robot.subsystems.tof;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotTracker;

public class ToFIOSim implements ToFIO {
  private double[] frontLeft = new double[] {0, 0};
  private double[] backLeft = new double[] {0, 0};
  private double[] frontRight = new double[] {0, 0};
  private double[] backRight = new double[] {0, 0};

  public ToFIOSim() {

  }

  @Override
  public void updateInputs(ToFIOInputs inputs) {
    inputs.frontLeft[0] = frontLeft[0];
    inputs.frontLeft[1] = frontLeft[1];
    inputs.backLeft[0] = backLeft[0];
    inputs.backLeft[1] = backLeft[1];
    inputs.frontRight[0] = frontRight[0];
    inputs.frontRight[1] = frontRight[1];
    inputs.backRight[0] = backRight[0];
    inputs.backRight[1] = backRight[1];

    RobotTracker.getInstance().recordToF(frontLeft, backLeft, frontRight, backRight);
  }

  public void simulateFrontLeft(double distanceInMm) {
    frontLeft[0] = distanceInMm;
    frontLeft[1] = Timer.getTimestamp();
  }

  public void simulateBackLeft(double distanceInMm) {
    backLeft[0] = distanceInMm;
    backLeft[1] = Timer.getTimestamp();
  }

  public void simulateFrontRight(double distanceInMm) {
    frontRight[0] = distanceInMm;
    frontRight[1] = Timer.getTimestamp();
  }

  public void simulateBackRight(double distanceInMm) {
    backRight[0] = distanceInMm;
    backRight[1] = Timer.getTimestamp();
  }
}
