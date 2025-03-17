package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

public class GyroIOSim extends GyroIONavX {
  public GyroIOSim() {
  }

  public SimDeviceSim getGyroDeviceSim() {
    // return new SimDeviceSim("navX-Sensor", navX.getPort());
    return new SimDeviceSim("navX-Sensor", 0);
  }
}
