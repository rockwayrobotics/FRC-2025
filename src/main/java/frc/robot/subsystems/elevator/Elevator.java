package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeightMeters = 0;
  private double heightMeters = 0;
  protected final double MIN_HEIGHT_METERS = 0;
  protected final double MAX_HEIGHT_METERS = 0.5;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    heightMeters = inputs.positionMeters;

    if (DriverStation.isDisabled()) {
      io.stop();
    } else {
      io.moveTowardsGoal(goalHeightMeters, heightMeters);
    }
  }

  public void setGoalHeightMeters(double heightMeters) {
    goalHeightMeters = MathUtil.clamp(heightMeters, MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);
  }

  public double getHeightMeters() {
    return heightMeters;
  }

  public double getGoalHeightMeters() {
    return goalHeightMeters;
  }

  public boolean atGoal() {
    // within 1 centimeter
    return Math.abs(heightMeters - goalHeightMeters) < 0.01;
  }

  public void stop() {
    io.stop();
  }
}
