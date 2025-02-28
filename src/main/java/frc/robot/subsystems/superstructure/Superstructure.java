package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;

public class Superstructure extends SubsystemBase{
  public final Elevator elevator;
  public final Chute chute;
  public final Grabber grabber;

  public Superstructure(Elevator elevator, Chute chute, Grabber grabber) {
    this.elevator = elevator;
    this.chute = chute;
    this.grabber = grabber;
  }

  @Override
  public void periodic() {
    elevator.periodic();
    chute.periodic();
    grabber.periodic();
  }

  public void setElevatorGoalHeightMeters(double heightMeters) {
    elevator.setGoalHeightMeters(heightMeters);
  }

  public void setChutePivotGoalRads(double pivotAngleRads) {
    chute.setPivotGoalRads(pivotAngleRads);
  }

  public void startShooting() {
    chute.startShooting();
  }

  public void stopShooting() {
    chute.stopShooting();
  }

  // FIXME: Add grabber methods too
}
