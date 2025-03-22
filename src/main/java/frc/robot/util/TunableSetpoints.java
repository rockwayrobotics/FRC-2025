package frc.robot.util;

public class TunableSetpoints {
  // Intake:
  private final Tuner intake_elevator_height_mm = new Tuner("Scoring/Intake/elevator_height_mm", 415, true);
  private final Tuner intake_chute_pivot_angle_rads = new Tuner("Scoring/Intake/chute_pivot_angle_rads", 0.809, true);

  // Scoring:
  // L1:
  private final Tuner L1_elevator_height_mm = new Tuner("Scoring/L1/elevator_height_mm", 377, true);
  private final Tuner L1_chute_pivot_angle_rads = new Tuner("Scoring/L1/chute_pivot_angle_rads", 1.55085, true);
  private final Tuner L1_shoot_speed_normalized = new Tuner("Scoring/L1/shoot_speed_normalized", 0.5, true);

  // L2:
  private final Tuner L2_elevator_height_mm = new Tuner("Scoring/L2/elevator_height_mm", 765, true);
  private final Tuner L2_chute_pivot_angle_rads = new Tuner("Scoring/L2/chute_pivot_angle_rads", 1.22, true);
  private final Tuner L2_shoot_speed_normalized = new Tuner("Scoring/L2/shoot_speed_normalized", 0.5, true);

  // L3:
  private final Tuner L3_elevator_height_mm = new Tuner("Scoring/L3/elevator_height_mm", 1160, true);
  private final Tuner L3_chute_pivot_angle_rads = new Tuner("Scoring/L3/chute_pivot_angle_rads", 1.28, true);
  private final Tuner L3_shoot_speed_normalized = new Tuner("Scoring/L3/shoot_speed_normalized", 0.5, true);

  // L2 Algae:
  private final Tuner algae_L2_elevator_height_mm = new Tuner("Scoring/Algae/L2/elevator_height_mm", 740, true);
  private final Tuner algae_L2_wrist_angle_rads = new Tuner("Scoring/Algae/L2/wrist_angle_rads", 0.273, true);

  // L3 Algae:
  private final Tuner algae_L3_elevator_height_mm = new Tuner("Scoring/Algae/L3/elevator_height_mm", 1170, true);
  private final Tuner algae_L3_wrist_angle_rads = new Tuner("Scoring/Algae/L3/wrist_angle_rads", 0.297, true);

  // Floor Algae:
  private final Tuner algae_floor_elevator_height_mm = new Tuner("Scoring/Algae/Floor/elevator_height_mm", 10, true);
  private final Tuner algae_floor_wrist_angle_rads = new Tuner("Scoring/Algae/Floor/wrist_angle_rads", 0.18,
      true);

  // Score Algae:
  private final Tuner algae_score_elevator_height_mm = new Tuner("Scoring/Algae/Score/elevator_height_mm", 200, true);
  private final Tuner algae_score_wrist_angle_rads = new Tuner("Scoring/Algae/Score/wrist_angle_rads", 0,
      true);

  // Barge Algae:
  private final Tuner barge_prepare_elevator_height_mm = new Tuner("Scoring/Algae/Barge/prepare_elevator_height_mm",
      200, true);
  private final Tuner barge_prepare_wrist_angle_rads = new Tuner("Scoring/Algae/Barge/prepare_wrist_angle_rads", -0.8,
      true);
  private final Tuner barge_shoot_elevator_height_mm = new Tuner("Scoring/Algae/Barge/shoot_elevator_height_mm", 1200,
      true);
  private final Tuner barge_shoot_wrist_angle_rads = new Tuner("Scoring/Algae/Barge/shoot_wrist_angle_rads", -1.3,
      true);
  private final Tuner barge_shoot_elevator_speed = new Tuner("Scoring/Algae/Barge/shoot_elevator_speed", 0.95, true);
  private final Tuner barge_when_shoot_elevator_height_mm = new Tuner(
      "Scoring/Algae/Barge/when_shoot_elevator_height_mm", 890, true);
  private final Tuner barge_when_wrist_angle_height_mm = new Tuner("Scoring/Algae/Barge/when_wrist_angle_height_mm",
      250, true);

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

  public double L1_shoot_speed_normalized() {
    return L1_shoot_speed_normalized.get();
  }

  public double L2_elevator_height_mm() {
    return L2_elevator_height_mm.get();
  }

  public double L2_chute_pivot_angle_rads() {
    return L2_chute_pivot_angle_rads.get();
  }

  public double L2_shoot_speed_normalized() {
    return L2_shoot_speed_normalized.get();
  }

  public double L3_elevator_height_mm() {
    return L3_elevator_height_mm.get();
  }

  public double L3_chute_pivot_angle_rads() {
    return L3_chute_pivot_angle_rads.get();
  }

  public double L3_shoot_speed_normalized() {
    return L3_shoot_speed_normalized.get();
  }

  // Algae getters:
  public double algae_L2_elevator_height_mm() {
    return algae_L2_elevator_height_mm.get();
  }

  public double algae_L2_wrist_angle_rads() {
    return algae_L2_wrist_angle_rads.get();
  }

  public double algae_L3_elevator_height_mm() {
    return algae_L3_elevator_height_mm.get();
  }

  public double algae_L3_wrist_angle_rads() {
    return algae_L3_wrist_angle_rads.get();
  }

  public double algae_floor_elevator_height_mm() {
    return algae_floor_elevator_height_mm.get();
  }

  public double algae_floor_wrist_angle_rads() {
    return algae_floor_wrist_angle_rads.get();
  }

  public double algae_score_elevator_height_mm() {
    return algae_score_elevator_height_mm.get();
  }

  public double algae_score_wrist_angle_rads() {
    return algae_score_wrist_angle_rads.get();
  }

  public double barge_prepare_elevator_height_mm() {
    return barge_prepare_elevator_height_mm.get();
  }

  public double barge_prepare_wrist_angle_rads() {
    return barge_prepare_wrist_angle_rads.get();
  }

  public double barge_shoot_elevator_height_mm() {
    return barge_shoot_elevator_height_mm.get();
  }

  public double barge_shoot_wrist_angle_rads() {
    return barge_shoot_wrist_angle_rads.get();
  }

  public double barge_shoot_elevator_speed() {
    return barge_shoot_elevator_speed.get();
  }

  public double barge_when_shoot_elevator_height_mm() {
    return barge_when_shoot_elevator_height_mm.get();
  }

  public double barge_when_wrist_angle_height_mm() {
    return barge_when_wrist_angle_height_mm.get();
  }
}
