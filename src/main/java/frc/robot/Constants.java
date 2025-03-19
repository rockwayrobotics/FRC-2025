// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class NT {
    public static final String CHUTE_MODE = "/Pi/chute_mode";
    public static final String TOF_MODE = "/Pi/tof_mode";
    public static final String CORNERS = "/Pi/Corners";
    public static final String TS_DIST_MM = "/Pi/ts_dist_mm";
  }

  public static final class Field {
    public static final double REEF_CORNER_TO_NEAR_POST_METERS = 0.3061 + 0.008;
    public static final double REEF_CORNER_TO_FAR_POST_METERS = 0.3286 + REEF_CORNER_TO_NEAR_POST_METERS;
  }

  public static final class Gamepads {
    public static final int DRIVER = 0;
    public static final int OPERATOR_1 = 1;
    public static final int OPERATOR_2 = 2;

    // Since the default joystick has negative as forward, this scale is negative
    // when MASTER_ORIENTATION_REVERSED is false; you should probably never have to
    // change this, swap the CAN ids instead.
    public static final double JOY_SPEED_SCALE = -1;
    // to be tested
    public static final double JOY_ROTATE_SCALE = -1;
  }

  public static final class Dimensions {
    // Dimensions of frame without bumpers
    public static final double FRAME_X_LENGTH_METERS = 0.816;
    public static final double FRAME_Y_WIDTH_METERS = 0.66;
  }

  public static final class CAN {
    // to switch the foward direction of the robot, switch the CAN ids for the left
    // and right drive
    // NEO with Spark MAX
    public static final int LEFT_DRIVE_MOTOR_R = 1;
    public static final int LEFT_DRIVE_MOTOR_F = 2;
    public static final int RIGHT_DRIVE_MOTOR_R = 3;
    public static final int RIGHT_DRIVE_MOTOR_F = 4;

    // NEO with Spark MAX
    public static final int CLIMP_MOTOR = 5;

    // NEO with Spark MAX
    public static final int ELEVATOR_MOTOR_BACK = 6;
    public static final int ELEVATOR_MOTOR_FRONT = 12;

    // NEO with Spark MAX
    public static final int GRABBER_LEFT_MOTOR = 7;
    public static final int GRABBER_RIGHT_MOTOR = 8;
    public static final int GRABBER_WRIST_MOTOR = 9;

    // NEO with Spark MAX
    public static final int PIVOT_MOTOR = 10;
    public static final int SHOOTER_MOTOR = 11;
  }

  public static final class Chute {
    // We start at -90 degrees with the shooter facing to starboard
    // 0 degrees is vertically down
    // 90 degrees is the other limit, facing flat to port
    public static final double PIVOT_INITIAL_ANGLE_RADS = Units.degreesToRadians(-90.0);

    public static final double PIVOT_TROUGH = Units.degreesToRadians(-60);
    public static final double PIVOT_L2 = Units.degreesToRadians(-60);
    public static final double PIVOT_L3 = Units.degreesToRadians(-60);
    public static final double PIVOT_LOAD = Units.degreesToRadians(-60);
    public static final double PIVOT_FLAT = Units.degreesToRadians(-90);

    public static final double PIVOT_GEAR_RATIO = 45; // 60/12 and 9:1 planetary

    // FIXME: Complete guess
    public static final double MOI = 0.1;
    public static final double CHUTE_LENGTH_METERS = 0.66;
    // Distance from loading end of chute to center of wheel, measured on y-axis
    // only
    public static final double CHUTE_WHEEL_POSITION_METERS = 0.568;
    public static final double SHOOTER_WHEEL_RADIUS_METERS = 0.05715 / 2.0;
    // Thickness of chute along robot drive axis
    public static final double CHUTE_X_THICKNESS_METERS = 0.107;
    // Measured with 0 as the center of the robot frame, positive x forwards
    public static final double CHUTE_CENTER_X_POSITION_METERS = Dimensions.FRAME_X_LENGTH_METERS / 2 - 0.276
        + CHUTE_X_THICKNESS_METERS / 2;

    public static final double CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM = 256;
  }

  public static final class Climp {
    public static final double PIVOT_GEAR_RATIO = 540;
  }

  public static final class Drive {
    // Our target speed for scoring in m/s
    public static final double SCORING_SPEED = 0.45;

    public static final double TRACK_WIDTH_METERS = 0.56;
    public static final double MAX_SPEED_MPS = 4.;
    public static final double MAX_ACCEL_MPSS = 2.;

    public final static double WHEEL_CIRCUM_CM = 46.8;
    public final static double WHEEL_RADIUS_METERS = WHEEL_CIRCUM_CM / 100 / 2 / Math.PI;
    public final static double WHEEL_GEAR_RATIO = 8.46;
    public final static double LEFT_SCALING = (1509 / 1501.89) / 100;
    public final static double RIGHT_SCALING = (1509 / 1504.21) / 100;
    public final static double WHEEL_ENCODER_SCALING = WHEEL_CIRCUM_CM / WHEEL_GEAR_RATIO;

    // Measured with the feedForwardCharacterization drive command
    //                                         Mar 14 Simple        SysId
    public static final double REAL_KS = 0.23357; // 0.24487 // left 0.23191 right 0.23573
    public static final double REAL_KV = 2.22866; // 2.36262 // left 2.2005 right 2.2267
    // left kA 0.66129
    // right kA 0.59644
    public static final double SIM_KS = 0.08776;
    public static final double SIM_KV = 2.27678;

    // you probably never need to change this, swap the ids for left and right motor
    // groups.
    public final static boolean LEFT_DRIVE_INVERTED = false;
    public final static boolean RIGHT_DRIVE_INVERTED = true;

    public final static double rotation_kP = 0.3;
  }

  public static final class Elevator {
    // Predefined heights in millimeters
    public static final double TROUGH_HEIGHT = 1000.0;
    public static final double L2_HEIGHT = 1000.0;
    public static final double L3_HEIGHT = 1000.0;
    public static final double LOAD_HEIGHT = 1000.0;
    public static final double ALGAE_L2_HEIGHT = 1000.0;
    public static final double ALGAE_L3_HEIGHT = 1000.0;
    public static final double CLIMB_HEIGHT = 0.0;

    public static final double GEAR_RATIO = 2;

    public static final double ELEVATOR_CONVERSION_FACTOR = 26.3472615411;

    public static final double SPROCKET_DIAMETER_METERS = 0.0448060837;
    // March 1st 2025:
    // (375/1.33203125) = 281.5249266862
    // (1192/4.251) = 280.4046106798

    public static final double CARRIAGE_MASS_KG = 5;
    // Minimum height of pivot center
    public static final double MIN_PIVOT_HEIGHT_METERS = 0.24;
    // Maximum extension height of elevator
    public static final double MAX_HEIGHT_METERS = 1.320;
  }

  public static final class Grabber {
    public static final boolean LEFT_GRABBER_INVERTED = false;
    public static final boolean RIGHT_GRABBER_INVERTED = true;
    // Wrist speed radians per second must be a positive value
    public static final double WRIST_SPEED_RADIANS_PER_SECOND = 0.5;

    public static final double WRIST_CONVERSION_FACTOR = 0.0831945515;
    /** Algae acquired if getVolts is over this value */
    public static final double ALGAE_DISTANCE_SENSOR_ACQUIRED_VOLTS = 2.5;
  }

  public static final class ToFSensor {
    // FIXME: Simulated locations of sensors are not measured
    // DO NOT USE THESE ARE WRONG
    public static final Transform2d FRONT_LEFT = new Transform2d(new Translation2d(0.5, 0.5),
        Rotation2d.fromDegrees(90));
    public static final Transform2d BACK_LEFT = new Transform2d(new Translation2d(-0.5, 0.5),
        Rotation2d.fromDegrees(90));
    public static final Transform2d FRONT_RIGHT = new Transform2d(new Translation2d(0.5, -0.5),
        Rotation2d.fromDegrees(-90));
    public static final Transform2d BACK_RIGHT = new Transform2d(new Translation2d(-0.5, -0.5),
        Rotation2d.fromDegrees(-90));

    // Distances measured from centre of ToF sensors to fore and aft edges of the chute,
    // averaged to give a reasonable value for the centre. Note that we aren't certain
    // the centre is the right thing yet as the coral appears possibly to pop towards
    // the rear as it leaves the green spinner wheels, so there may be a little room for
    // refinement here.
    public static final double TOF_FWD_RIGHT_TO_CHUTE = (321 + 201) / 2; // 261mm
    public static final double TOF_FWD_LEFT_TO_CHUTE = (320 + 195) / 2; // 257.5mm

    public static final double TOF_TO_BUMPER = 20.0; // mm
  }

  public static enum ToFSensorLocation {
    NONE_SELECTED, FRONT_LEFT, BACK_LEFT, FRONT_RIGHT, BACK_RIGHT
  }

  public static enum ReefBar {
    NEAR,
    FAR
  }

  public static final class Reef {
    // Note: while this says measured from field CAD, it's unclear what this
    // is measured to.  Is it the top of the bar, the bottom, the centre of
    // the "face" at the end, or something else?  We measured manually to
    // the centre of the face (which seems to be the best reference point)
    // and got more like 50mm.
    public static final double TIP_TO_WALL = 53.6; // measured from field CAD

    public static final double CORNER_TO_NEAR_POST_METERS = 0.3061;
    public static final double CORNER_TO_FAR_POST_METERS = 0.6347;
  }

  public static final class Digital {
    public static final int CHUTE_SHOOT_CORAL_BEAMBREAK = 0;
    public static final int CHUTE_LOAD_CORAL_BEAMBREAK = 1;
    public static final int ALGAE_DISTANCE_SENSOR = 2;
    public static final int ALGAE_HOME_LIMIT_SWITCH = 5;
    public static final int ELEVATOR_HOME_BEAMBREAK = 6;
    public static final int CHUTE_HOME_LIMIT_SWITCH = 7;
  }

  public static final class PathPlanner {
    // FIXME: This needs to be measured!
    public static final double ROBOT_MASS_KG = Kilograms.convertFrom(112.0, Pounds);
    // FIXME: This needs to be measured!
    public static final double ROBOT_MOI = 7.5;
    public static final RobotConfig CONFIG = new RobotConfig(
        ROBOT_MASS_KG,
        ROBOT_MOI,
        new ModuleConfig(
            Drive.WHEEL_RADIUS_METERS,
            Drive.MAX_SPEED_MPS,
            1.0, // FIXME: Wheel coefficient of friction should be measured!
            DCMotor.getNEO(2).withReduction(Drive.WHEEL_GEAR_RATIO), // FIXME: Check this
            38, // FIXME: Current limit should be a constant
            2),
        Drive.TRACK_WIDTH_METERS);
  }

  // Constants for LEDs
  public static final class LED {
    public final static int LED_PWM = 0;
    public final static int LED_LENGTH = 58;

    public static enum modes {
      Rainbow,
      Red,
      Green,
      Blue,
      Purple,
      Orange,
      Yellow,
      Off,
      BreathingYellow,
      BreathingMagenta,
      FlashingOrange,
      FlashingGreen,
      badApple,
      heatGradient,
      whiteDotLines,
    }
  }

  public static enum CoralLevel {
    L1, L2, L3, Intake
  }

  public static enum AlgaeLevel {
    Floor, Score, L2, L3
  }

  public static enum Side {
    LEFT, RIGHT
  }
}
