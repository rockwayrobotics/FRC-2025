package frc.robot.sim;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants.Drive;

/**
 * Since REVPhysicsSim doesn't work, let's try to implement
 * our own simulation that updates the relevant simulation
 * variables for Spark MAX motors.
 */
public class DrivebaseSim {
  // Simulation value handles that are doubles
  public static final String APPLIED_OUTPUT = "Applied Output";
  public static final String VELOCITY = "Velocity";
  public static final String VELOCITY_CONVERSION_FACTOR = "Velocity Conversion Factor";
  public static final String POSITION = "Position";
  public static final String POSITION_CONVERSION_FACTOR = "Position Conversion Factor";
  public static final String BUS_VOLTAGE = "Bus Voltage"; // Always 12.0?
  public static final String MOTOR_CURRENT = "Motor Current";
  public static final String ANALOG_VOLTAGE = "Analog Voltage";
  public static final String ANALOG_VELOCITY = "Analog Velocity";
  public static final String ANALOG_POSITION = "Analog Position";
  public static final String ALT_ENCODER_VELOCITY = "Alt Encoder Velocity";
  public static final String ALT_ENCODER_POSITION = "Alt Encoder Position";
  public static final String STALL_TORQUE = "Stall Torque";
  public static final String FREE_SPEED = "Free Speed";

  // Simulation value handles that are ints
  public static final String FAULTS = "Faults";
  public static final String STICKY_FAULTS = "Sticky Faults";
  public static final String MOTOR_TEMPERATURE = "Motor Temperature"; // Always 25?
  public static final String CONTROL_MODE = "Control Mode";
  public static final String FW_VERSION = "FW Version";

  private SparkMax m_leftMotor;
  private SparkMax m_rightMotor;

  private int m_leftMotorDeviceHandle;
  private int m_rightMotorDeviceHandle;

  private int m_leftMotorEncoderDeviceHandle;
  private int m_rightMotorEncoderDeviceHandle;

  private SimDouble m_leftAppliedOutput;
  private SimDouble m_rightAppliedOutput;

  private SimDouble m_leftPositionMeters;
  private SimDouble m_rightPositionMeters;

  private SimDouble m_leftVelocityMetersPerSecond;
  private SimDouble m_rightVelocityMetersPerSecond;

  private SimDouble m_leftEncoderPositionMeters;
  private SimDouble m_rightEncoderPositionMeters;

  private SimDouble m_leftEncoderVelocityMetersPerSecond;
  private SimDouble m_rightEncoderVelocityMetersPerSecond;

  // Suggested code from
  // https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
  public static final String GYRO_DEVICE_NAME = "navX-Sensor[4]";
  private SimDouble m_yaw;

  private DifferentialDrivetrainSim m_drivetrainSim;

  public DrivebaseSim(SparkMax leftMotor, SparkMax rightMotor) {
    m_drivetrainSim = new DifferentialDrivetrainSim(
        DCMotor.getNEO(2),
        Drive.WHEEL_GEAR_RATIO,
        // FIXME: These values are defaults from
        // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
        // and really should be measured.
        7.5,
        112.0,
        // This value is the wheel radius in metres
        Drive.WHEEL_CIRCUM / 100. / Math.PI / 2.,
        Drive.TRACK_WIDTH_METERS,
        // TODO: Add noise to the simulation here as standard deviation values for
        // noise:
        // x, y in m
        // heading in rad
        // l/r velocity m/s
        // l/r position in m
        VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));

    m_leftMotor = leftMotor;
    m_rightMotor = rightMotor;

    m_leftMotorDeviceHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + leftMotor.getDeviceId() + "]");
    m_rightMotorDeviceHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + rightMotor.getDeviceId() + "]");

    m_leftMotorEncoderDeviceHandle = SimDeviceDataJNI
        .getSimDeviceHandle("SPARK MAX [" + leftMotor.getDeviceId() + "] RELATIVE ENCODER");
    m_rightMotorEncoderDeviceHandle = SimDeviceDataJNI
        .getSimDeviceHandle("SPARK MAX [" + rightMotor.getDeviceId() + "] RELATIVE ENCODER");

    m_leftAppliedOutput = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_leftMotorDeviceHandle, DrivebaseSim.APPLIED_OUTPUT));
    m_rightAppliedOutput = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_rightMotorDeviceHandle, DrivebaseSim.APPLIED_OUTPUT));

    m_leftPositionMeters = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_leftMotorDeviceHandle, POSITION));
    m_rightPositionMeters = new SimDouble(SimDeviceDataJNI.getSimValueHandle(m_rightMotorDeviceHandle, POSITION));
    m_leftVelocityMetersPerSecond = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_leftMotorDeviceHandle, VELOCITY));
    m_rightVelocityMetersPerSecond = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_rightMotorDeviceHandle, VELOCITY));

    m_leftEncoderPositionMeters = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_leftMotorEncoderDeviceHandle, POSITION));
    m_rightEncoderPositionMeters = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_rightMotorEncoderDeviceHandle, POSITION));

    m_leftEncoderVelocityMetersPerSecond = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_leftMotorEncoderDeviceHandle, VELOCITY));

    m_rightEncoderVelocityMetersPerSecond = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(m_rightMotorEncoderDeviceHandle, VELOCITY));

    int gyroDeviceHandle = SimDeviceDataJNI.getSimDeviceHandle(GYRO_DEVICE_NAME);
    m_yaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroDeviceHandle, "Yaw"));
  }

  public void disable() {
    // m_drivetrainSim.setState(new Matrix<>(Nat.N7(), Nat.N1()));
    // m_leftVelocityMetersPerSecond.set(0);
    // m_rightVelocityMetersPerSecond.set(0);
  }

  public void reset(Pose2d pose) {
    // Reset the drivetrain simulator to be at the new position, but keep the
    // original facing.
    // Since we do not reset the gyro in the drivebase subsystem, we have to keep
    // the same heading
    // here even after reset.
    Pose2d originalFacingPose = new Pose2d(pose.getTranslation(),
        new Rotation2d(m_drivetrainSim.getHeading().getRadians()));
    m_drivetrainSim.setPose(originalFacingPose);
  }

  private void updateAppliedOutput() {
    m_leftAppliedOutput.set(m_leftMotor.get() * m_leftMotor.getBusVoltage());
    m_rightAppliedOutput.set(m_rightMotor.get() * m_rightMotor.getBusVoltage());
  }

  public void update(double period) {
    this.updateAppliedOutput();
    m_drivetrainSim.setInputs(m_leftMotor.getAppliedOutput(), m_rightMotor.getAppliedOutput());
    m_drivetrainSim.update(period);

    m_leftPositionMeters.set(m_drivetrainSim.getLeftPositionMeters());
    m_rightPositionMeters.set(m_drivetrainSim.getRightPositionMeters());
    m_leftVelocityMetersPerSecond.set(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightVelocityMetersPerSecond.set(m_drivetrainSim.getRightVelocityMetersPerSecond());

    m_leftEncoderPositionMeters.set(m_drivetrainSim.getLeftPositionMeters());
    m_rightEncoderPositionMeters.set(m_drivetrainSim.getRightPositionMeters());

    m_leftEncoderVelocityMetersPerSecond.set(m_drivetrainSim.getLeftVelocityMetersPerSecond());
    m_rightEncoderVelocityMetersPerSecond.set(m_drivetrainSim.getRightVelocityMetersPerSecond());

    m_yaw.set(m_drivetrainSim.getHeading().getDegrees());
  }

  public static void printAllSimulationValues() {
    DrivebaseSim.printSimulationValues("");
  }

  public static void printSimulationValues(String prefix) {
    var deviceInfo = SimDeviceDataJNI.enumerateSimDevices(prefix);
    for (var device : deviceInfo) {
      var values = SimDeviceDataJNI.enumerateSimValues(device.handle);
      for (var value : values) {
        switch (value.value.getType()) {
          case HALValue.kBoolean:
            System.out.println(device.name + "." + value.name + ": (boolean): " + value.value.getBoolean());
            break;
          case HALValue.kDouble:
            System.out.println(device.name + "." + value.name + ": (double): " + value.value.getDouble());
            break;
          case HALValue.kEnum:
            System.out.println(device.name + "." + value.name + ": (enum): " + value.value.getLong());
            break;
          case HALValue.kInt:
            System.out.println(device.name + "." + value.name + ": (int): " + value.value.getLong());
            break;
          case HALValue.kLong:
            System.out.println(device.name + "." + value.name + ": (long): " + value.value.getLong());
            break;
          case HALValue.kUnassigned:
            System.out.println(device.name + "." + value.name + ": (unassigned)");
            break;
          default:
            System.out.println(device.name + "." + value.name + ": (unknown): " + value.value.getLong());
        }
      }
    }
  }
}
