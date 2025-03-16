
import frc.robot.Constants.Reef;
import frc.robot.Constants.ReefBar;
import frc.robot.Constants.ToFSensor;
import frc.robot.Constants.ToFSensorLocation;
import frc.robot.Constants.Field;
// package frc.robot.util;

// import java.util.EnumSet;
// import java.util.function.Consumer;

/**
 * A class for calculating scoring parameters (e.g. run and shot distances)
 * based on data from encoder, Pi TOF sensors, etc.
 *
 */
public class ShotCalc {

  private double pos;
  private double dist;
  private ReefBar bar;
  private ToFSensorLocation tof;
  private double corner_to_post; // distance from corner to target post
  private double tof_to_chute; // distance from tof sensor to chute centre

  private double run_to_target = 0.0;
  private double dist_to_target = 0.0;

  public ShotCalc(double pos, double dist, ReefBar bar, ToFSensorLocation tof) {
    this.pos = pos;
    this.dist = dist - ToFSensor.TOF_TO_BUMPER;
    this.bar = bar;
    this.tof = tof;

    if (bar == ReefBar.NEAR) {
      this.corner_to_post = Reef.CORNER_TO_NEAR_POST_METERS * 1000.0;
    }
    else {
      this.corner_to_post = Reef.CORNER_TO_FAR_POST_METERS * 1000.0;
    }

    switch (tof) {
      case NONE_SELECTED: this.tof_to_chute = 0.0;
      case FRONT_LEFT: this.tof_to_chute = ToFSensor.TOF_FWD_LEFT_TO_CHUTE;
      case BACK_LEFT: this.tof_to_chute = 0.0;
      case FRONT_RIGHT: this.tof_to_chute = ToFSensor.TOF_FWD_RIGHT_TO_CHUTE;
      case BACK_RIGHT: this.tof_to_chute = 0.0;
    }
  }

  // Do the math, and report back with a tuple containing the
  // encoder position at the target (lined up to shoot) and
  // the expected orthogonal distance (from chute to branch tip).
  public void update(double pos, double dist) {
    dist -= ToFSensor.TOF_TO_BUMPER;

    // calculate current angle estimate
    double delta_pos = pos - this.pos;
    double delta_dist = dist - this.dist;
    // angle in radians, positive is moving away from wall
    double angle = Math.atan2(delta_dist, delta_pos);

    // calculate position where chute will be aligned with target
    double run_fore = this.corner_to_post * Math.cos(angle);
    double run_aft = Reef.TIP_TO_WALL * Math.sin(angle);
    double run_adjusted = run_fore - run_aft;
    double run_total = this.tof_to_chute + run_adjusted;
    // print(f'pos={pos:.0f} dist={dist:.0f} -> angle={angle*180/math.pi:.1f}'
    //     f', run: fwd ({run_fore:.0f}) - aft ({run_aft:.0f}) = {run_adjusted:.0f}'
    //     f', run_total={run_total:.0f}')

    // calculate distance from frame to tip once we're at the target position
    double dist_to_wall = this.dist + run_adjusted * Math.tan(angle);
    double wall_to_tip = Reef.TIP_TO_WALL / Math.cos(angle);
    double shot_dist = dist_to_wall + wall_to_tip;
    
    this.run_to_target = this.pos + run_total;
    this.dist_to_target = shot_dist;
  }

  public double getTargetPos() {
    return this.run_to_target;
  }

  public double getTargetDist() {
    return this.dist_to_target;
  }
}
