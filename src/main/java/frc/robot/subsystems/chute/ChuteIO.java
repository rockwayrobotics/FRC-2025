package frc.robot.subsystems.chute;

import java.util.concurrent.CompletableFuture;

import org.littletonrobotics.junction.AutoLog;

public interface ChuteIO {
  @AutoLog
  public static class CoralIOInputs {
    public boolean homeSwitchPressed = false;
    // TODO: Figure out proper coordinate system
    public double pivotAngleRadians = 0.0;
    public double pivotVelocityRadPerSec = 0.0;
    public double pivotSetpoint = 0.0;
    public boolean coralLoading = false;
    public boolean coralReady = false;
    public double shooterVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(CoralIOInputs inputs) {
  }

  public default void moveTowardsPivotGoal(double goalAngleRadians, double currentAngleRadians) {
  }

  public default void stopPivot() {
  }

  public default void setBrakeMode(boolean mode) {
  }

  public default CompletableFuture<Boolean> home() {
    return null;
  }

  /**
   * Set the shooter to a normalized speed in [-1, 1].
   * FIXME: This is probably not what we want
   * 
   * @param speed
   */
  public default void setShooterSpeed(double speed) {
  }
}
