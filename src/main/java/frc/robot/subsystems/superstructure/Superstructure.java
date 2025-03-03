package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
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
    // FIXME: Check if elevator is homed. If not... do nothing?
    // FIXME: Check chute state, if not safe, move it first? Or do nothing?
    elevator.setGoalHeightMillimeters(heightMillimeters);
  }

  public void setChutePivotGoalRads(double pivotAngleRads) {
    // FIXME: Check if chute is homed. If not... do nothing?
    // FIXME: Check elevator height, if not safe, move it first? Or do nothing?
    chute.setPivotGoalRads(pivotAngleRads);
  }

  public void startShooting() {
    chute.startShooting();
  }

  public void stopShooting() {
    chute.stopShooting();
  }

  // FIXME: Add grabber methods too

  /**
   * Schedules the superstructure homing sequence.
   */
  public void home() {
    Commands.parallel(
        // Commands.sequence(
        // Commands.runOnce(() -> elevator.home()),
        // Commands.waitUntil(() -> elevator.isHomed()),
        // Commands.runOnce(() -> elevator.setGoalHeightMillimeters(400))),
        Commands.sequence(
            // Commands
            // .waitUntil(() -> elevator.getGoalHeightMillimeters() >
            // Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM
            // && elevator.atGoal()),
            // // wait for elevator to be at 300mm / at least 280mm
            Commands.runOnce(() -> chute.home())),
        Commands.runOnce(() -> grabber.home()))
        .schedule();
  }

  /**
   * Returns (but does not schedule) a command to shut down the superstructure
   * carefully. This is intended to be used to get ready for climp.
   */
  public Command foldForClimp() {
    return Commands.parallel(
        Commands.runOnce(() -> chute.setPivotGoalRads(0)),
        Commands.runOnce(() -> grabber.setWristGoalRads(0)),
        Commands.sequence(
            Commands.waitUntil(() -> chute.getPivotAngleRads() < Units.degreesToRadians(3)),
            Commands.runOnce(() -> elevator.setGoalHeightMillimeters(0))),
        Commands.sequence(
            Commands.waitUntil(() -> elevator.getHeightMillimeters() < 10),
            Commands.runOnce(() -> {
              // FIXME: Unclear what disable means. Stop?
              // chute.disable();
              // grabber.disable();
              // elevator.disable();
              // FIXME: Set state as ready for climp?
            })));
  }
}
