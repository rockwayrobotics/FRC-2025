// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final class CAN {
    // to switch the foward direction of the robot, switch the CAN ids for the left
    // and right drive
    public static final int LEFT_DRIVE_MOTOR_R = 1;
    public static final int LEFT_DRIVE_MOTOR_F = 2;
    public static final int RIGHT_DRIVE_MOTOR_R = 3;
    public static final int RIGHT_DRIVE_MOTOR_F = 4;
    public static final int LEFT_INTAKE = 5; // brushed
    public static final int RIGHT_INTAKE = 6; // brushed one opposite direction
    public static final int BELT = 7; // indexer
    public static final int GEAR = 8; // angle
    public static final int LEFT_FLYWHEEL = 9; // negative on one flywheel?
    public static final int RIGHT_FLYWHEEL = 10; // negative on one flywheel?
    public static final int CLIMB = 11;
  }

  public static final class Digital {
    public final static int CLIMB_HOME_SWITCH = 0;
    public final static int SHOOTER_TOP_SENSOR = 1;
    public final static int INTAKE_LOAD_SENSOR = 2;
  }

  public static final class Drive {
    public static final double TRACK_WIDTH_METERS = 0.56;
    public static final double MAX_SPEED_MPS = 4.;

    public final static double WHEEL_CIRCUM = 46.8;
    public final static double WHEEL_GEAR_RATIO = 8.46;
    public final static double LEFT_SCALING = (1509 / 1501.89) / 100;
    public final static double RIGHT_SCALING = (1509 / 1504.21) / 100;
    public final static double WHEEL_ENCODER_SCALING = WHEEL_CIRCUM / WHEEL_GEAR_RATIO;

    // you probably never need to change this, swap the ids for left and right motor
    // groups.
    public final static boolean LEFT_DRIVE_INVERTED = false;
    public final static boolean RIGHT_DRIVE_INVERTED = true;

    public final static double rotation_kP = 0.3;
  }

  public static final class Climber {
    public final static float REVERSE_MAX = 0;
    public final static float FORWARD_MAX = 270;
  }

  public static final class Shooter {
    public static enum ScoringMode {
      SPEAKER,
      AMP,
    }
  }

  public static final class Angler {
    public final static float ANGLE_TOP_MAX = 0f;
    public final static float ANGLE_BOTTOM_MAX = 40f;

    public final static double AUTO_SPEAKER_SETPOINT = 13.5;
    public final static double SPEAKER_SETPOINT = 0;
    public final static double ZERO_SETPOINT = 0;
    public final static double HALF_CYCLE_SETPOINT = 25;
    public final static double LOW_HALF_CYCLE_SETPOINT = 40;

    public final static double DEFAULT_PID_kP = 0.3;
    public final static double DEFAULT_PID_kI = 0;
    public final static double DEFAULT_PID_kD = 0;

    public final static double DEFAULT_POSITIVE_CLAMP = 0.15;
    public final static double DEFAULT_NEGATIVE_CLAMP = -0.15;
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
