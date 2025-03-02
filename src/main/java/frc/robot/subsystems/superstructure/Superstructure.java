package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;

public class Superstructure extends SubsystemBase {
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

  public void setElevatorGoalHeightMillimeters(double heightMillimeters) {
    elevator.setGoalHeightMillimeters(heightMillimeters);
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

  public void home() {
    Commands.parallel(
        // Commands.sequence(
        //     Commands.runOnce(() -> elevator.home()),
        //     Commands.waitUntil(() -> elevator.isHomed()),
        //     Commands.runOnce(() -> elevator.setGoalHeightMillimeters(400))),
        Commands.sequence(
            // Commands
            //     .waitUntil(() -> elevator.getGoalHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM
            //         && elevator.atGoal()),
            // // wait for elevator to be at 300mm / at least 280mm
            Commands.runOnce(() -> chute.home())),
        Commands.runOnce(() -> grabber.home()))
        .schedule();
  }
}
