// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.system.plant.DCMotor;

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
    public static final int ELEVATOR_MOTOR_LEFT = 5;
    public static final int ELEVATOR_MOTOR_RIGHT = 6;

    // NEO with Spark MAX
    public static final int PIVOT_MOTOR = 7;
    public static final int SHOOTER_MOTOR = 8;

    // Vortex with Spark Flex
    public static final int CLIMP_MOTOR = 9;

    // NEO with Spark MAX
    public static final int GRABBER_WRIST_MOTOR = 10;
    public static final int GRABBER_LEFT_MOTOR = 11;
    public static final int GRABBER_RIGHT_MOTOR = 12;
  }

  public static final class Chute {
    public static final double PIVOT_GEAR_RATIO = 45; // 60/12 and 9:1 planetary

    // FIXME: Complete guess
    public static final double MOI = 0.1;
    public static final double CHUTE_LENGTH_METERS = 0.66;
    // Distance from loading end of chute to center of wheel, measured on y-axis only
    public static final double CHUTE_WHEEL_POSITION_METERS = 0.568;
    public static final double SHOOTER_WHEEL_RADIUS_METERS = 0.05715 / 2.0;
    // Thickness of chute along robot drive axis
    public static final double CHUTE_X_THICKNESS_METERS = 0.107;
    // Measured with 0 as the center of the robot frame, positive x forwards
    public static final double CHUTE_CENTER_X_POSITION_METERS = Dimensions.FRAME_X_LENGTH_METERS / 2 - 0.276 + CHUTE_X_THICKNESS_METERS / 2;
  }

  public static final class Drive {
    public static final double TRACK_WIDTH_METERS = 0.56;
    public static final double MAX_SPEED_MPS = 4.;
    public static final double MAX_ACCEL_MPSS = 2.;

    public final static double WHEEL_CIRCUM_CM = 46.8;
    public final static double WHEEL_RADIUS_METERS = WHEEL_CIRCUM_CM / 100 / 2 / Math.PI;
    public final static double WHEEL_GEAR_RATIO = 8.46;
    public final static double LEFT_SCALING = (1509 / 1501.89) / 100;
    public final static double RIGHT_SCALING = (1509 / 1504.21) / 100;
    public final static double WHEEL_ENCODER_SCALING = WHEEL_CIRCUM_CM / WHEEL_GEAR_RATIO;

    // you probably never need to change this, swap the ids for left and right motor
    // groups.
    public final static boolean LEFT_DRIVE_INVERTED = false;
    public final static boolean RIGHT_DRIVE_INVERTED = true;

    public final static double rotation_kP = 0.3;
  }

  public static final class Elevator {
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
  }

  public static final class Digital {
    public static final int CORAL_LOAD_BEAMBREAK = 0;
    public static final int CORAL_SHOOT_BEAMBREAK = 1;
    public static final int ALGAE_HOME_SWITCH = 2;
  }

  public static final class Analog {
    public static final int ALGAE_DISTANCE_SENSOR = 0;
  }

  public static final class PathPlanner {
    // FIXME: This needs to be measured!
    public static final double ROBOT_MASS_KG = 112.0;
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
