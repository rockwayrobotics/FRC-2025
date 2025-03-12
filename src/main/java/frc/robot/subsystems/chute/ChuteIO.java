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
    public double appliedOutput = 0.0;
  }

  public default void updateInputs(CoralIOInputs inputs) {
  }

  public default void moveTowardsPivotGoal(double goalAngleRadians, double currentAngleRadians) {
  }

  public default void stopPivot() {
  }

  public default void setBrakeMode(boolean mode) {
  }

  public default void setEncoder(double position) {
  }

  public default CompletableFuture<Boolean> home() {
    return null;
  }
}
