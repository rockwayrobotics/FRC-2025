// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Radians;

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
    public static final String SENSOR_MODE = "/Pi/SensorMode";
    public static final String CORNERS = "/Pi/Corners";
  }

  public static final class Field {
    public static final double REEF_CORNER_TO_NEAR_POST_METERS = 0.3061;
    public static final double REEF_CORNER_TO_FAR_POST_METERS = 0.6347;
  }

  public static final class Gamepads {
    public static final int DRIVER = 0;
    public static final int OPERATOR = 1;

    // Since the default joystick has negative as forward, this scale is negative
    // when MASTER_ORIENTATION_REVERSED is false; you should probably never have to
    // change this, swap the CAN ids instead.
    public static final double JOY_SPEED_SCALE = -1;
    // to be tested
    public static final double JOY_ROTATE_SCALE = -0.3;
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

    // Vortex with Spark Flex
    public static final int CLIMP_MOTOR = 5;

    // Vortex with Spark Flex
    public static final int ELEVATOR_MOTOR = 6;

    // NEO with Spark MAX
    public static final int GRABBER_RIGHT_MOTOR = 7;
    public static final int GRABBER_LEFT_MOTOR = 8;
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
    public static final double REAL_KS = 0.18389;
    public static final double REAL_KV = 2.26057;
    public static final double SIM_KS = 0.08776;
    public static final double SIM_KV = 2.27678;

    // you probably never need to change this, swap the ids for left and right motor
    // groups.
    public final static boolean LEFT_DRIVE_INVERTED = false;
    public final static boolean RIGHT_DRIVE_INVERTED = true;

    public final static double rotation_kP = 0.3;
  }

  public static final class Elevator {
    // Predefined heights in meters
    public static final double TROUGH_HEIGHT = 1.0;
    public static final double L2_HEIGHT = 2.0;
    public static final double L3_HEIGHT = 3.0;
    public static final double LOAD_HEIGHT = 4.0;
    public static final double ALGAE_L2_HEIGHT = 2.0;
    public static final double ALGAE_L3_HEIGHT = 3.0;
    public static final double CLIMB_HEIGHT = 0.0;

    public static final double SPROCKET_RADIUS_METERS = 0.042;
    public static final double GEAR_RATIO = 2;
    public static final double CARRIAGE_MASS_KG = 5;
    // Minimum height of pivot center
    public static final double MIN_PIVOT_HEIGHT_METERS = 0.24;
    // Maximum extension height of elevator - FIXME: Not measured
    public static final double MAX_HEIGHT_METERS = 1.5;
  }

  public static final class Grabber {
    public static final boolean LEFT_GRABBER_INVERTED = false;
    public static final boolean RIGHT_GRABBER_INVERTED = true;
    // Wrist speed radians per second must be a positive value
    public static final double WRIST_SPEED_RADIANS_PER_SECOND = 0.5;
    public static final double WRIST_GEAR_RATIO = 81; // 1: 9 * 9
  }

  public static final class ToFSensor {
    // FIXME: Simulated locations of sensors are not measured
    public static final Transform2d FRONT_LEFT = new Transform2d(new Translation2d(0.5, 0.5),
        Rotation2d.fromDegrees(90));
    public static final Transform2d BACK_LEFT = new Transform2d(new Translation2d(-0.5, 0.5),
        Rotation2d.fromDegrees(90));
    public static final Transform2d FRONT_RIGHT = new Transform2d(new Translation2d(0.5, -0.5),
        Rotation2d.fromDegrees(-90));
    public static final Transform2d BACK_RIGHT = new Transform2d(new Translation2d(-0.5, -0.5),
        Rotation2d.fromDegrees(-90));
  }

  public static final class Digital {
    public static final int ALGAE_HOME_LIMIT_SWITCH = 1; // normally open
    public static final int CHUTE_HOME_LIMIT_SWITCH = 2; // normally open, when intake is to the left it is clsoed
    // 3 is empty
    public static final int CHUTE_SHOOT_CORAL_BEAMBREAK = 4;
    public static final int CHUTE_LOAD_CORAL_BEAMBREAK = 5;
    public static final int ELEVATOR_HOME_BEAMBREAK = 6;
  }

  public static final class Analog {
    public static final int ALGAE_DISTANCE_SENSOR = 3;
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
}
