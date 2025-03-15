package frc.robot.subsystems.grabber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GrabberIOSim implements GrabberIO {
  //protected final SingleJointedArmSim wristSim;
  //protected final PIDController wristPid = new PIDController(0.8, 0, 0);
  //private double wristVoltage = 0;

  public GrabberIOSim() {
  }
    
  @Override
  public void updateInputs(GrabberIOInputs inputs) {

  }
}