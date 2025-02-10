package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  protected final ElevatorSim sim;
  protected final PIDController controller = new PIDController(0.5, 0, 0);
  private double inputVoltage = 0;

  public ElevatorIOSim(double startHeightMeters) {
    sim = new ElevatorSim(DCMotor.getNeoVortex(2), Constants.Elevator.GEAR_RATIO, Constants.Elevator.CARRIAGE_MASS_KG,
        Constants.Elevator.DRUM_RADIUS_METERS, 0.0, Constants.Elevator.MAX_HEIGHT_METERS, false, 0.0);
    sim.setState(startHeightMeters, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputVoltage = controller.calculate(sim.getVelocityMetersPerSecond());
    sim.setInputVoltage(inputVoltage);

    sim.update(0.02);
    inputs.appliedVoltage = inputVoltage;
    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setGoal(double positionMeters) {
    controller.setSetpoint(positionMeters);
  }

  @Override
  public void stop() {
    inputVoltage = 0;
    sim.setInputVoltage(inputVoltage);
  }

  public double getChutePivotHeightMeters() {
    return sim.getPositionMeters() + Constants.Elevator.MIN_PIVOT_HEIGHT_METERS;
  }
}
