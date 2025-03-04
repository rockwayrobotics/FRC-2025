package frc.robot.util;

public class TunableSetpoints {
  // Intake:
  private final Tuner intake_elevator_height_mm = new Tuner("Scoring/Intake/elevator_height_mm", 500, true);
  private final Tuner intake_chute_pivot_angle_rads = new Tuner("Scoring/Intake/chute_pivot_angle_rads", 1, true);

  // Scoring:
  // L1:
  private final Tuner L1_elevator_height_mm = new Tuner("Scoring/L1/elevator_height_mm", 500, true);
  private final Tuner L1_chute_pivot_angle_rads = new Tuner("Scoring/L1/chute_pivot_angle_rads", 1, true);

  // L2:
  private final Tuner L2_elevator_height_mm = new Tuner("Scoring/L2/elevator_height_mm", 750, true);
  private final Tuner L2_chute_pivot_angle_rads = new Tuner("Scoring/L2/chute_pivot_angle_rads", 0.8, true);

  // L3:
  private final Tuner L3_elevator_height_mm = new Tuner("Scoring/L3/elevator_height_mm", 1000, true);
  private final Tuner L3_chute_pivot_angle_rads = new Tuner("Scoring/L3/chute_pivot_angle_rads", 0.8, true);

  public double intake_elevator_height_mm() {
    return intake_elevator_height_mm.get();
  }

  public double intake_chute_pivot_angle_rads() {
    return intake_chute_pivot_angle_rads.get();
  }

  public double L1_elevator_height_mm() {
    return L1_elevator_height_mm.get();
  }

  public double L1_chute_pivot_angle_rads() {
    return L1_chute_pivot_angle_rads.get();
  }

  public double L2_elevator_height_mm() {
    return L2_elevator_height_mm.get();
  }

  public double L2_chute_pivot_angle_rads() {
    return L2_chute_pivot_angle_rads.get();
  }

  public double L3_elevator_height_mm() {
    return L3_elevator_height_mm.get();
  }

  public double L3_chute_pivot_angle_rads() {
    return L3_chute_pivot_angle_rads.get();
  }
}
