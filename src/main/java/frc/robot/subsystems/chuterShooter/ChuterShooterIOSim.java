package frc.robot.subsystems.chuterShooter;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ChuterShooterIOSim implements ChuterShooterIO {
  private final Object motorLock = new Object();
  protected final FlywheelSim shooterSim;

  private double shooterSpeed = 0;
  private boolean coralLoading = false;
  private boolean coralReady = false;

  // Null callbacks do not get called.
  private Notifier delayedShootNotifier = new Notifier(null);

  public ChuterShooterIOSim() {
    // Not sure if this is actually a flywheel, and these are random numbers
    shooterSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNEO(1), 0.2, 10 / 18.0),
        DCMotor.getNEO(1));
  }

  @Override
  public void updateInputs(ChuterShooterIOInputs inputs) {
    synchronized (motorLock) {
      shooterSim.setInputVoltage(12.0 * shooterSpeed);
    }
    shooterSim.update(0.02);
    shooterSim.getAngularVelocityRadPerSec();

    inputs.shooterVelocityRadPerSec = shooterSim.getAngularVelocityRadPerSec();
    inputs.coralLoading = coralLoading;
    inputs.coralReady = coralReady;
  }

  @Override
  public void setShooterSpeed(double speed) {
    shooterSpeed = speed;
  }

  @Override
  public void stopShooting() {
    this.shooterSpeed = 0;
    delayedShootNotifier.stop();
  }

  @Override
  public void scheduleShoot(double speed, double delaySeconds) {
    delayedShootNotifier.setCallback(() -> {
      synchronized (motorLock) {
        shooterSim.setInputVoltage(12.0 * speed);
      }
    });
    delayedShootNotifier.startSingle(delaySeconds);
    setShooterSpeed(speed);
  }

  public void setCoralLoading(boolean loading) {
    coralLoading = loading;
  }

  public void setCoralReady(boolean ready) {
    coralLoading = ready;
  }

  public double getShooterVelocityRadPerSec() {
    return shooterSim.getAngularVelocityRadPerSec();
  }
}
