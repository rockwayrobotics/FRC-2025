package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.Interlock;
import frc.robot.util.Tuner;

public class Elevator {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private double goalHeightMillimeters = 0;
  private double heightMillimeters = 0;
  private boolean isHomed = false;

  final Tuner elevatorMaxVelocityMmPerSecond = new Tuner("Elevator/speed_max_mm_per_sec", 100, true);
  final Tuner elevatorMaxAccelMmPerSec2 = new Tuner("Elevator/accel_max_mm_per_sec2", 100, true);
  protected TrapezoidProfile motionProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(elevatorMaxVelocityMmPerSecond.get(), elevatorMaxAccelMmPerSec2.get()));

  final Interlock unlocked = new Interlock("Elevator");

  public Elevator(ElevatorIO io) {
    this.io = io;
    unlocked.addListener((e) -> {
      if (e.valueData.value.getBoolean()) {
        setGoalHeightMillimeters(heightMillimeters);
      }
    });

    elevatorMaxVelocityMmPerSecond.addListener((_e) -> updateMotionProfile());
    elevatorMaxAccelMmPerSec2.addListener((_e) -> updateMotionProfile());
  }

  private void updateMotionProfile() {
    motionProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(elevatorMaxVelocityMmPerSecond.get(), elevatorMaxAccelMmPerSec2.get()));
  }

  public void periodic(double minHeight) {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    heightMillimeters = inputs.positionMillimeters;
    Logger.recordOutput("Elevator/goal_height_mm", goalHeightMillimeters);
    Logger.recordOutput("Elevator/isHomed", isHomed);

    double allowedGoalHeightMm = goalHeightMillimeters;
    if (goalHeightMillimeters < minHeight) {
      allowedGoalHeightMm = minHeight;
    }

    if (DriverStation.isDisabled() || !isHomed) {
      io.stop();
      goalHeightMillimeters = allowedGoalHeightMm;
    } else {
      TrapezoidProfile.State nextGoal = motionProfile.calculate(0.02,
          new TrapezoidProfile.State(heightMillimeters, inputs.velocityMillimetersPerSec),
          new TrapezoidProfile.State(goalHeightMillimeters, 0));
      io.moveTowardsGoal(nextGoal.position, heightMillimeters);
    }
  }

  public void setGoalHeightMillimeters(double heightMillimeters) {
    goalHeightMillimeters = heightMillimeters;
  }

  public double getHeightMillimeters() {
    return heightMillimeters;
  }

  public double getGoalHeightMillimeters() {
    return goalHeightMillimeters;
  }

  public boolean atGoal() {
    // within 1 centimeter
    return Math.abs(heightMillimeters - goalHeightMillimeters) < 10;
  }

  public void stop() {
    io.stop();
  }

  public void setMaxNormalizedSpeedTuner(double speed) {
    io.setMaxNormalizedSpeedTuner(speed);
  }

  public double getMaxNormalizedSpeedTuner() {
    return io.getMaxNormalizedSpeedTuner();
  }

  public void home() {
    io.zeroEncoder();
    setGoalHeightMillimeters(0);
    isHomed = true;
  }

  public void stayStill() {
    setGoalHeightMillimeters(heightMillimeters);
  }
}
