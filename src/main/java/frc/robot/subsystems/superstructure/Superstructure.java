package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.util.Tuner;

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
    if (elevator.getGoalHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
      elevator.periodic();
    } else if ((Math.abs(chute.getPivotAngleRads()) - Units.degreesToRadians(90)) < 0.1) {
      System.out.println("passed check");
      elevator.periodic();
    }
    chute.periodic();
    grabber.periodic();
  }

  public void setElevatorGoalHeightMillimeters(double heightMillimeters) {
    // FIXME: Check if elevator is homed. If not... do nothing?
    // FIXME done?????: Check chute state, if not safe, move it first? Or do
    // nothing?
    if (heightMillimeters < Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
      if (chute.getPivotAngleRads() <= 0) {
        chute.setPivotGoalRads(Units.degreesToRadians(-90));
      } else {
        chute.setPivotGoalRads(Units.degreesToRadians(90));
      }
    }
    // ; // FIXME: make constant
    elevator.setGoalHeightMillimeters(MathUtil.clamp(heightMillimeters, 0, 1200 - 1));
  }

  public double getElevatorHeightMillimeters() {
    return elevator.getHeightMillimeters();
  }

  public void setChutePivotGoalRads(double pivotAngleRads) {
    // FIXME: Check if chute is homed. If not... do nothing?
    // FIXME: Check elevator height, if not safe, move it first? Or do nothing?
    chute.setPivotGoalRads(pivotAngleRads);
  }

  public double getPivotAngleRads() {
    return chute.getPivotAngleRads();
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
    var command = Commands.parallel(Commands.runOnce(() -> grabber.home()),
        Commands.runOnce(() -> elevator.setGoalHeightMillimeters(400)),
        Commands.sequence(
            Commands
                .waitUntil(() -> elevator.getHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM),
            Commands.runOnce(() -> chute.home())));

    command.addRequirements(this);
    command.schedule();
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
