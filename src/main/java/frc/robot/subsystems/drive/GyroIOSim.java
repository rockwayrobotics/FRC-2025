package frc.robot.subsystems.drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;

public class GyroIOSim implements GyroIO {
  private final DoubleSupplier yawSupplier;

  public GyroIOSim(DoubleSupplier yawSupplier) {
    // FIXME: Extend GyroIONavX and simulate the device as well.
    this.yawSupplier = yawSupplier;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromDegrees(yawSupplier.getAsDouble());
    inputs.yawVelocityRadPerSec = 0; // FIXME: Not sure how to simulate this
  }
}
