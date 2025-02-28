package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  protected final ElevatorSim sim;
  protected final PIDController controller = new PIDController(12, 0, 0);
  private double inputVoltage = 0;

  public ElevatorIOSim(double startHeightMeters) {
    sim = new ElevatorSim(DCMotor.getNeoVortex(2), Constants.Elevator.GEAR_RATIO, Constants.Elevator.CARRIAGE_MASS_KG,
        Constants.Elevator.SPROCKET_RADIUS_METERS, 0.0, Constants.Elevator.MAX_HEIGHT_METERS, false, 0.0);
    sim.setState(startHeightMeters, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputVoltage = controller.calculate(sim.getPositionMeters());
    sim.setInputVoltage(inputVoltage);

    sim.update(0.02);
    inputs.appliedVoltage = inputVoltage;
    inputs.positionMeters = sim.getPositionMeters();
    inputs.velocityMetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.homed = inputs.positionMeters < 0.001;
  }

  @Override
  public void moveTowardsGoal(double goalHeightMeters, double currentHeightMeters) {
    controller.setSetpoint(goalHeightMeters);
  }

  @Override
  public void stop() {
    inputVoltage = 0;
    sim.setInputVoltage(inputVoltage);
  }

  public double getChutePivotHeightMeters() {
    return sim.getPositionMeters();
  }
}
