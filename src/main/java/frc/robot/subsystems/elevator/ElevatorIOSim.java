package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.Constants;

public class ElevatorIOSim implements ElevatorIO {
  protected final ElevatorSim sim;
  protected final PIDController controller = new PIDController(12, 0, 0);
  private double inputVoltage = 0;

  public ElevatorIOSim(double startHeightMeters) {
    sim = new ElevatorSim(DCMotor.getNeoVortex(2), Constants.Elevator.GEAR_RATIO, Constants.Elevator.CARRIAGE_MASS_KG,
        Constants.Elevator.SPROCKET_DIAMETER_METERS, 0.0, Constants.Elevator.MAX_HEIGHT_METERS, false, 0.0);
    sim.setState(startHeightMeters, 0.0);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputVoltage = controller.calculate(sim.getPositionMeters());
    sim.setInputVoltage(inputVoltage);

    sim.update(0.02);
    inputs.appliedVoltage = inputVoltage;
    inputs.positionMillimeters = Millimeters.convertFrom(sim.getPositionMeters(), Meters);
    inputs.velocityMillimetersPerSec = sim.getVelocityMetersPerSecond();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.homeBeamBroken = inputs.positionMillimeters < 1;
  }

  @Override
  public void moveTowardsGoal(TrapezoidProfile.State state) {
    controller.setSetpoint(Meters.convertFrom(state.position, Millimeters));
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
