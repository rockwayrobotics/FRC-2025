package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AlgaeLevel;
import frc.robot.Constants.CoralLevel;
import frc.robot.Constants.Side;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.util.Sensors;
import frc.robot.util.TunableSetpoints;
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
      elevator.periodic();
    }

    // FIXME: make this more robust
    chute.periodic(elevator.getHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM - 50);

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

  public boolean isElevatorAtGoal() {
    return elevator.atGoal();
  }

  public boolean isPivotAtGoal() {
    return chute.isPivotAtGoal();
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

  public void setShooterMotor(double speed) {
    chute.setShooterMotor(speed);
  }

  public void stopShooting() {
    chute.stopShooting();
  }

  // FIXME: Add grabber methods too

  public void setWristGoalRads(double wristAngleRads) {
    grabber.setWristGoalRads(wristAngleRads);
  }

  public double getGrabberWristAngleRads() {
    return grabber.getCurrentRads();
  }

  public void setGrabberMotor(double speed) {
    grabber.setGrabberMotor(speed);
  }

  /**
   * Schedules the superstructure homing sequence.
   */
  public void home() {
    // var command = Commands.parallel(
    // Commands.runOnce(() -> grabber.home()),
    // Commands.runOnce(() -> {
    // elevator.home();
    // elevator.setGoalHeightMillimeters(400);
    // }),
    // Commands.sequence(
    // Commands
    // .waitUntil(() -> elevator.getHeightMillimeters() >
    // Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM),
    // Commands.runOnce(() -> chute.home())))
    // .finallyDo(() -> elevator.setGoalHeightMillimeters(0));

    // command.addRequirements(this);
    // command.schedule();
    var command = Commands.parallel(
        Commands.runOnce(() -> {
          elevator.home();
        }),
        Commands.runOnce(() -> {
          chute.home();
        }),
        Commands.runOnce(() -> {
          grabber.home();
        }));

    command.addRequirements(this);
    command.schedule();
  }

  /**
   * Returns (but does not schedule) a command to shut down the superstructure
   * carefully. This is intended to be used to get ready for climp.
   */
  public void foldForClimp() {
    Commands.parallel(
        Commands.runOnce(() -> grabber.setWristGoalRads(Units.degreesToRadians(-90))),
        Commands.runOnce(
            () -> {
              if (chute.getPivotAngleRads() <= 0) {
                chute.setPivotGoalRads(Units.degreesToRadians(-90));
              } else {
                chute.setPivotGoalRads(Units.degreesToRadians(90));
              }
            }),
        Commands.runOnce(() -> {
          if (elevator.getGoalHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM + 1) {
            elevator.setGoalHeightMillimeters(Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM + 1);
          }
        }),
        Commands.sequence(
            Commands.waitUntil(() -> Math.abs(chute.getPivotAngleRads()) > Units.degreesToRadians(87)),
            Commands.runOnce(() -> elevator.setGoalHeightMillimeters(0))))
        .schedule();
  }

  TunableSetpoints setpoints = new TunableSetpoints();

  public void gotoSetpoint(CoralLevel level, Side side) {
    int sideMultiplier = (side == Side.LEFT) ? -1 : 1;
    switch (level) {
      case L1:
        setChutePivotGoalRads(sideMultiplier * setpoints.L1_chute_pivot_angle_rads());
        setElevatorGoalHeightMillimeters(setpoints.L1_elevator_height_mm());
        break;
      case L2:
        setChutePivotGoalRads(sideMultiplier * setpoints.L2_chute_pivot_angle_rads());
        setElevatorGoalHeightMillimeters(setpoints.L2_elevator_height_mm());
        break;
      case L3:
        setChutePivotGoalRads(sideMultiplier * setpoints.L3_chute_pivot_angle_rads());
        setElevatorGoalHeightMillimeters(setpoints.L3_elevator_height_mm());
        break;
      case Intake:
        setChutePivotGoalRads(sideMultiplier * setpoints.intake_chute_pivot_angle_rads());
        setElevatorGoalHeightMillimeters(setpoints.intake_elevator_height_mm());
        break;
    }
  }

  public void gotoAlgaeSetpoint(AlgaeLevel level) {
    // Runnable suck = () -> {
    //   Commands.run(() -> grabber.setGrabberMotor(-0.75), this).onlyWhile(() -> Sensors.getInstance()
    //       .getGrabberAcquiredDistance() < Constants.Grabber.ALGAE_DISTANCE_SENSOR_ACQUIRED_VOLTS)
    //       .finallyDo(() -> grabber.setGrabberMotor(0)).schedule();
    // };
    switch (level) {
      case Floor:
        setElevatorGoalHeightMillimeters(setpoints.algae_floor_elevator_height_mm());
        setWristGoalRads(setpoints.algae_floor_wrist_angle_rads());
        // suck.run();
        break;
      case L2:
        setElevatorGoalHeightMillimeters(setpoints.algae_L2_elevator_height_mm());
        setWristGoalRads(setpoints.algae_L2_wrist_angle_rads());
        // suck.run();
        break;
      case L3:
        setElevatorGoalHeightMillimeters(setpoints.algae_L3_elevator_height_mm());
        setWristGoalRads(setpoints.algae_L3_wrist_angle_rads());
        // suck.run();
        break;
      case Score:
        setElevatorGoalHeightMillimeters(setpoints.algae_score_elevator_height_mm());
        setWristGoalRads(setpoints.algae_score_wrist_angle_rads());
        break;
    }
  }

  public void stayStill() {
    elevator.stayStill();
    chute.stayStill();
    grabber.stayStill();
  }
}
