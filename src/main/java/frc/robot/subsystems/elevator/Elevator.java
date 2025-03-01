package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Interlock;
import frc.robot.util.Tuner;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeightMeters = 0;
  private double heightMeters = 0;
  private boolean homed = false;

  final Tuner elevatorSoftLimitMin = new Tuner("Elevator/soft_limit_min", 0.4, true);
  final Tuner elevatorSoftLimitMax = new Tuner("Elevator/soft_limit_max", 0.6, true);
  final Interlock enabled = new Interlock("Elevator");

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    heightMeters = inputs.positionMeters;
    homed = inputs.homed;

    if (DriverStation.isDisabled() || !enabled.get()) {
      io.stop();
    } else {
      io.moveTowardsGoal(goalHeightMeters, heightMeters);
    }
  }

  public void setGoalHeightMeters(double heightMeters) {
    if (enabled.get()) {
      goalHeightMeters = MathUtil.clamp(heightMeters, elevatorSoftLimitMin.get(), elevatorSoftLimitMax.get());
    }
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

  public boolean isHomed() {
    return homed;
  }
}
