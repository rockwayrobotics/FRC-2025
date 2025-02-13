package frc.robot.subsystems.tof;

import edu.wpi.first.networktables.NetworkTableInstance;

public class ToFIOPi5 implements ToFIO {
  public ToFIOPi5() {
    NetworkTableInstance.getDefault().getTable("Pi5");
    // For now, assume that the Pi will send us data from all of the ToF sensors all of the time.

  }

  public void updateInputs(ToFIOInputs inputs) {

  }
}
