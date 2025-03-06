package frc.robot.subsystems.climp;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Interlock;

public class Climp extends SubsystemBase {
  private final ClimpIO io;
  private final ClimpIOInputsAutoLogged inputs = new ClimpIOInputsAutoLogged();

  final Interlock unlocked = new Interlock("Climp");

  private double climpGoalRads = 0;

  public Climp(ClimpIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climp", inputs);

    /**
     * FIXME: Should the climp be fully disabled until something happens
     * when it's close to the end of the match?
     */
    // FIXME: INTERLOCK NOT CHECKED
    if (DriverStation.isDisabled()) {
      io.stop();
    }
    // else {
    // io.moveTowardsGoal(climpGoalRads, inputs.angleRadians);
    // }
    // FIXME FIXME FIXME FIXME THIS DOESNT HAVE A PID OR ANY LIMITS
  }

  /**
   * Set speed of climp motor
   *
   * @param speed in range -1 to 1
   */
  public void setNormalizedSpeed(double speed) {
    io.setNormalizedSpeed(speed);
  }

  /**
   * Set climp goal angle in radians
   *
   * @param climpGoalRads
   */
  public void setClimpGoalRads(double climpGoalRads) {
    this.climpGoalRads = climpGoalRads;
  }

  public double getClimpAngleRads() {
    return inputs.angleRadians;
  }

  public void stayStill() {
    // could set a setpoint if we used one
    // since we don't, this won't resist any external force (except with brake mode)
    setNormalizedSpeed(0);
  }
}
