package frc.robot.subsystems.drive;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.hal.HALValue;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;

public class DriveIOSim extends DriveIOSparkMax {
  // Suggested code from
  // https://pdocs.kauailabs.com/navx-mxp/software/roborio-libraries/java/
  public static final String GYRO_DEVICE_NAME = "navX-Sensor[4]";

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

  private DifferentialDrivetrainSim drivetrain;

  private SparkMaxSim leftMotorSim;
  private SparkMaxSim rightMotorSim;

  private SimDouble leftAppliedOutput;
  private SimDouble rightAppliedOutput;

  private SimDouble yaw;
  private SimDouble yawVelocityRadPerSec;

  public DriveIOSim(GyroIOSim gyroIO) {
    super();
    drivetrain = new DifferentialDrivetrainSim(DCMotor.getNEO(2),
        Constants.Drive.WHEEL_GEAR_RATIO,
        // FIXME: These values are defaults from
        // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
        // and really should be measured.
        Constants.PathPlanner.ROBOT_MOI,
        Constants.PathPlanner.ROBOT_MASS_KG,
        Constants.Drive.WHEEL_RADIUS_METERS,
        Constants.Drive.TRACK_WIDTH_METERS,
        // TODO: Add noise to the simulation here as standard deviation values for
        // noise:
        // x, y in m
        // heading in rad
        // l/r velocity m/s
        // l/r position in m
        VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));
    
    leftMotorSim = new SparkMaxSim(leftDriveMotorF, DCMotor.getNEO(2));
    rightMotorSim = new SparkMaxSim(rightDriveMotorF, DCMotor.getNEO(2));

    int leftMotorDeviceHandle = SimDeviceDataJNI
        .getSimDeviceHandle("SPARK MAX [" + leftDriveMotorF.getDeviceId() + "]");
    int rightMotorDeviceHandle = SimDeviceDataJNI
        .getSimDeviceHandle("SPARK MAX [" + rightDriveMotorF.getDeviceId() + "]");

    leftAppliedOutput = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(leftMotorDeviceHandle, APPLIED_OUTPUT));
    rightAppliedOutput = new SimDouble(
        SimDeviceDataJNI.getSimValueHandle(rightMotorDeviceHandle, APPLIED_OUTPUT));

    yaw = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroIO.getGyroDeviceSim().getNativeHandle(), "Yaw"));
    yawVelocityRadPerSec = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroIO.getGyroDeviceSim().getNativeHandle(), "Velocity Z"));
  }

  public double getYaw() {
    return yaw.get();
  }

  public static void printAllSimulationValues() {
    printSimulationValues("");
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

    @Override
  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }

    @Override
  public void updateInputs(DriveIOInputs inputs) {
    leftAppliedOutput.set(leftDriveMotorF.get() * leftDriveMotorF.getBusVoltage());
    rightAppliedOutput.set(rightDriveMotorF.get() * rightDriveMotorF.getBusVoltage());

    drivetrain.setInputs(leftAppliedOutput.get(), rightAppliedOutput.get());
    drivetrain.update(0.02);

    leftMotorSim.iterate(drivetrain.getLeftVelocityMetersPerSecond(), leftDriveMotorF.getBusVoltage(), 0.02);
    rightMotorSim.iterate(drivetrain.getRightVelocityMetersPerSecond(), rightDriveMotorF.getBusVoltage(), 0.02);

    double lastYaw = yaw.get();
    yaw.set(drivetrain.getHeading().getDegrees());
    yawVelocityRadPerSec.set((yaw.get() - lastYaw) / 0.02);

    super.updateInputs(inputs);
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftDriveMotorF.setVoltage(leftVolts);
    rightDriveMotorF.setVoltage(rightVolts);
  }

  @Override
  public void setVelocity(double leftRadPerSec, double rightRadPerSec, double leftFFVolts, double rightFFVolts) {
    leftController.setReference(leftRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, leftFFVolts);
    rightController.setReference(rightRadPerSec, ControlType.kVelocity, ClosedLoopSlot.kSlot0, rightFFVolts);
  }
}
