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
    if (DriverStation.isDisabled() || !unlocked.get()) {
      io.stop();
    } else {
      io.moveTowardsGoal(climpGoalRads, inputs.angleRadians);
    }
  }

  /**
   * Set speed of climp motor
   * 
   * @param speed in range -1 to 1
   */
  public void setNormalizedSpeed(double speed) {
    if (unlocked.get()) {
      io.setNormalizedSpeed(speed);
    }
  }

  /**
   * Set climp goal angle in radians
   *
   * @param climpGoalRads
   */
  public void setClimpGoalRads(double climpGoalRads) {
    if (unlocked.get()) {
      this.climpGoalRads = climpGoalRads;
    }
  }
}
