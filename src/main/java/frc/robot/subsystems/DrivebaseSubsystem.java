package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.GenericEntry;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import frc.robot.Constants;
import frc.robot.Constants.Drive;
import frc.robot.sim.DrivebaseSim;

public class DrivebaseSubsystem extends SubsystemBase {

  SparkMax m_leftDriveMotorF;
  SparkMax m_leftDriveMotorR;
  SparkMax m_rightDriveMotorF;
  SparkMax m_rightDriveMotorR;

  private final DifferentialDrive m_drive;
  DifferentialDriveKinematics m_kinematics;

  RelativeEncoder m_leftDriveEncoder;
  RelativeEncoder m_rightDriveEncoder;

  private PIDController m_leftPid;
  private PIDController m_rightPid;

  SlewRateLimiter filter = new SlewRateLimiter(0);

  private double m_scale = 1;

  private DrivebaseSim m_drivebaseSim;

  public double distanceDrivenAuto;
  public double rotationScale;
  boolean isBrakeMode;
  boolean isCoastMode;

  GenericEntry rotationScaleWidget;
  GenericEntry brakeModeWidget;
  GenericEntry coastModeWidget;
  GenericEntry leftDistanceWidget;
  GenericEntry rightDistanceWidget;

  GenericEntry navxMonitorWidget;
  GenericEntry counterWidget;

  Field2d field = new Field2d();
  // GenericEntry fieldWidget;

  double counter = 0;

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  private double yawOffset;
  private DifferentialDriveOdometry driveOdometry;

  // Tracking of whether we have encoder reset issues.
  private int leftErrorCount = 0;
  private int rightErrorCount = 0;

  ShuffleboardTab dashboardTab = Shuffleboard.getTab("Drivebase");

  // Use PIDs in before calling tankDrive.
  private boolean m_experimentalPID = true;

  /////////////////////////////
  // Simulation variables - would be nice to remove or minimize these to ensure
  ///////////////////////////// there
  // is no impact on non-simulation performance.
  private boolean m_isSimulation = false;

  /**
   * Holds the last simulation time for duration calculation, matches
   * REVPhysicsSim
   */
  private long m_lastSimTime;

  /** Allows first-time initialization of last simulation time. */
  private boolean m_startedSimulation = false;
  /////////////////////////////

  public DrivebaseSubsystem(boolean isSimulation) {
    m_isSimulation = isSimulation;

    // Position is measured in motor revolutions by default, but we want metres.
    // Wheel encoder scaling gives us centimetres and takes the gear ratio into
    // account,
    // and the scaling values appear to take more accurate wheel measurements and cm
    // -> m
    // Distance conversion is the same as the position factor, but convert from
    // minutes to seconds
    // since velocity is measured in rpm by default but we want m/s.
    EncoderConfig encoderConfig = new EncoderConfig()
        .positionConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.LEFT_SCALING)
        .velocityConversionFactor(Drive.WHEEL_ENCODER_SCALING * Drive.LEFT_SCALING / 60);

    m_leftDriveMotorF = new SparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_F, MotorType.kBrushless);
    m_leftDriveMotorF.configure(
        new SparkMaxConfig().smartCurrentLimit(38).inverted(Constants.Drive.LEFT_DRIVE_INVERTED).apply(encoderConfig),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_leftDriveMotorR = new SparkMax(Constants.CAN.LEFT_DRIVE_MOTOR_R, MotorType.kBrushless);
    m_leftDriveMotorR.configure(new SparkMaxConfig().smartCurrentLimit(38).follow(m_leftDriveMotorF),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_rightDriveMotorF = new SparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_F, MotorType.kBrushless);
    m_rightDriveMotorF.configure(
        new SparkMaxConfig().smartCurrentLimit(38).inverted(Constants.Drive.RIGHT_DRIVE_INVERTED).apply(encoderConfig),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    m_rightDriveMotorR = new SparkMax(Constants.CAN.RIGHT_DRIVE_MOTOR_R, MotorType.kBrushless);
    m_rightDriveMotorR.configure(new SparkMaxConfig().smartCurrentLimit(38).follow(m_rightDriveMotorF),
        ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    // m_leftDriveMotorF.getPIDController().setP(0.2);
    // m_rightDriveMotorF.getPIDController().setP(0.2);

    m_drive = new DifferentialDrive(m_leftDriveMotorF, m_rightDriveMotorF);

    setDrivebaseIdle(IdleMode.kBrake);
    m_leftDriveEncoder = m_leftDriveMotorF.getEncoder();
    m_rightDriveEncoder = m_rightDriveMotorF.getEncoder();
    // when robot goes forward, left encoder spins positive and right encoder spins
    // negative

    m_leftDriveEncoder.setPosition(0);
    m_rightDriveEncoder.setPosition(0);

    // When we had no velocity, we liked kp=0.2
    m_leftPid = new PIDController(1.7, 0, 0);
    m_rightPid = new PIDController(1.7, 0, 0);

    rotationScaleWidget = dashboardTab.addPersistent("Driving Rotation Scale Factor", 0.76)
        .getEntry();

    brakeModeWidget = dashboardTab.add("Brake Mode", false).getEntry();
    coastModeWidget = dashboardTab.add("Coast Mode", false).getEntry();

    navxMonitorWidget = dashboardTab.add("NavX Monitor", 0).getEntry();
    counterWidget = dashboardTab.add("Counter Widget", 0).getEntry();

    leftDistanceWidget = dashboardTab.add("Left Distance", 0).getEntry();
    rightDistanceWidget = dashboardTab.add("Right Distance", 0).getEntry();

    // Note: the dashboard listens to changes on the field object
    // so we don't have to publish changes explicitly.
    dashboardTab.add("Field2d", field);
    m_kinematics = new DifferentialDriveKinematics(Drive.TRACK_WIDTH_METERS);
    driveOdometry = new DifferentialDriveOdometry(m_gyro.getRotation2d().unaryMinus(), getLDistance(), getRDistance());

    PathPlannerLogging.setLogActivePathCallback(this::saveActivePath);
    PathPlannerLogging.setLogTargetPoseCallback(this::saveTargetPose);

    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this::getCurrentSpeeds, // Current ChassisSpeeds supplier
        this::drive, // Method that will drive the robot given ChassisSpeeds
        new PPLTVController(0.02), // duration in seconds between update loop calls, defaults to 0.02s = 20ms
        new RobotConfig(112., 7.5,
            new ModuleConfig(
                Drive.WHEEL_CIRCUM / 2 / Math.PI,
                Drive.MAX_SPEED_MPS,
                1.0 /* placeholder for coefficient of friction */,
                DCMotor.getNEO(1).withReduction(Drive.WHEEL_GEAR_RATIO),
                38,
                2 /* is this 4? */),
            Drive.TRACK_WIDTH_METERS),
        () -> {
          // Boolean supplier that controls when the path will be mirrored for the red
          // alliance
          // This will flip the path being followed to the red side of the field.
          // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );
  }

  private void saveActivePath(List<Pose2d> activePath) {
    field.getObject("activePath").setPoses(activePath);
  }

  private void saveTargetPose(Pose2d target) {
    field.getObject("target").setPose(target);
  }

  public void setDrivebaseIdle(IdleMode setting) {
    var idleConfig = new SparkMaxConfig().idleMode(setting);
    m_rightDriveMotorF.configure(idleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_rightDriveMotorR.configure(idleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftDriveMotorF.configure(idleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    m_leftDriveMotorR.configure(idleConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void disable() {
    setDrivebaseIdle(IdleMode.kBrake);
    if (m_isSimulation) {
      m_drivebaseSim.disable();
    }
  }

  public void stop() {
    set(0, 0);
  }

  public void set(double speed, double rotation) {
    // speed = filter.calculate(speed);
    m_drive.curvatureDrive(speed * m_scale, rotation * m_scale, true);
  }

  public void setPathPlannerSpeed(ChassisSpeeds speeds) {
    DifferentialDriveWheelSpeeds wheelSpeeds = m_kinematics.toWheelSpeeds(speeds);
    wheelSpeeds.desaturate(Drive.MAX_SPEED_MPS);

    if (m_experimentalPID) {
      var leftOutput = m_leftPid.calculate(m_leftDriveEncoder.getVelocity(), wheelSpeeds.leftMetersPerSecond);
      var rightOutput = m_rightPid.calculate(m_rightDriveEncoder.getVelocity(), wheelSpeeds.rightMetersPerSecond);
      m_drive.tankDrive(leftOutput / Drive.MAX_SPEED_MPS, rightOutput / Drive.MAX_SPEED_MPS, false);
    } else {
      m_drive.tankDrive(wheelSpeeds.leftMetersPerSecond / Drive.MAX_SPEED_MPS,
          wheelSpeeds.rightMetersPerSecond / Drive.MAX_SPEED_MPS, false);
    }

  }

  public void setScale(double scale) {
    m_scale = scale;
  }

  // Get the pose of the robot as Pose2d
  public Pose2d getPose() {
    var pose = driveOdometry.getPoseMeters();
    return pose;
  }

  // Reset the Pose2d of the robot
  // This gets called if the path has an initial pose - which ours does.
  public void resetPose(Pose2d pose2d) {
    this.resetEncoders();
    if (m_isSimulation) {
      m_drivebaseSim.reset(pose2d);
    }
    this.driveOdometry.resetPosition(
        m_gyro.getRotation2d().unaryMinus(),
        getLDistance(),
        getRDistance(),
        pose2d);

    System.out.printf("Resetting pose to %s%n", pose2d.toString());
  }

  public ChassisSpeeds getCurrentSpeeds() {
    var wheelSpeeds = new DifferentialDriveWheelSpeeds(m_leftDriveEncoder.getVelocity(),
        m_rightDriveEncoder.getVelocity());
    var speeds = m_kinematics.toChassisSpeeds(wheelSpeeds);
    return speeds;
  }

  public void drive(ChassisSpeeds speeds) {
    setPathPlannerSpeed(speeds);
  }

  /**
   * Gets the distance travelled by the left-side wheels of the drivebase since
   * last reset.
   *
   * @return Distance, in meters.
   */
  public double getLDistance() {
    return m_leftDriveEncoder.getPosition();
  }

  /**
   * Gets the distance travelled by the right-side wheels of the drivebase since
   * last reset.
   *
   * @return Distance in meters.
   */
  public double getRDistance() {
    return m_rightDriveEncoder.getPosition();
  }

  public void zeroGyro() {
    System.out.println("NavX Connected: " + m_gyro.isConnected());
    m_gyro.reset();
  }

  public void setAutoOffset(double autoOffset) {
    yawOffset = autoOffset;
  }

  public double getYaw() {
    return m_gyro.getYaw() + yawOffset;
  }

  public double getPitch() {
    return m_gyro.getPitch();
  }

  public double getRoll() {
    return m_gyro.getRoll();
  }

  public double getAngle() {
    return m_gyro.getAngle();
  }

  /** Resets drivebase encoder distances to 0. */
  public void resetEncoders() {
    // Troubleshooting for theorized CAN failure where our attempt
    // to reset an encoder fails.
    // We're trying both (before checking failures, so we can
    // track whether just one or both fail), and returning
    // if all is well, but otherwise counting the failures and
    // retrying. We'll print the stats as we change states.
    for (int i = 0; i < 5; i++) {
      // Reset but check that return code is good. In a future world
      // we can do more like recording the specific error code, but
      // since we're adding this mid-competition I want to be conservative.
      boolean leftOk = m_leftDriveEncoder.setPosition(0) == REVLibError.kOk;
      boolean rightOk = m_rightDriveEncoder.setPosition(0) == REVLibError.kOk;

      if (leftOk && rightOk) {
        break; // Success! Get us out of here.
      } else {
        if (!leftOk)
          leftErrorCount += 1;
        if (!rightOk)
          rightErrorCount += 1;
      }
    }
  }

  public void reportFailures(String state) {
    System.out.printf("Reset failures (%s): %d, %d%n",
        state, leftErrorCount, rightErrorCount);
  }

  @Override
  public void periodic() {
    driveOdometry.update(m_gyro.getRotation2d().unaryMinus(), getLDistance(), getRDistance());
    rotationScale = rotationScaleWidget.getDouble(0.76);
    leftDistanceWidget.setDouble(getLDistance());
    rightDistanceWidget.setDouble(getRDistance());

    // This publishes changes through to the dashboard.
    field.setRobotPose(driveOdometry.getPoseMeters());

    if (isBrakeMode != brakeModeWidget.getBoolean(false)) {
      isBrakeMode = brakeModeWidget.getBoolean(false);
      if (isBrakeMode) {
        setDrivebaseIdle(IdleMode.kBrake);
      }

      if (isCoastMode != coastModeWidget.getBoolean(false)) {
        isCoastMode = coastModeWidget.getBoolean(false);
        if (isCoastMode) {
          setDrivebaseIdle(IdleMode.kCoast);
        }
      }
    }
    navxMonitorWidget.setDouble(m_gyro.getAngle());

    counter++;

    if (counter == 50) {
      counter = 0;
    }
    counterWidget.setDouble(counter);
  }

  public void onSimulationInit() {
    m_drivebaseSim = new DrivebaseSim(m_leftDriveMotorF, m_rightDriveMotorF);
  }

  @Override
  public void simulationPeriodic() {
    // This code is basically a duplicate of the getPeriod method in
    // REVPhysicsSim.SimProfile.
    // It would be much better for debugging if we could simulate time passing
    // rather than
    // run it in realtime, but unfortunately we have at least two different timers
    // running
    // 1. Command scheduler timer
    // 2. REVPhysicsSim.SimProfile uses System.nanoTime
    // We want to ensure that our drivetrain simulator does its update with an
    // equivalent
    // duration, so it's copied here for now.
    if (!m_startedSimulation) {
      m_lastSimTime = System.nanoTime();
      m_startedSimulation = true;
    }
    long now = System.nanoTime();
    final double period = (now - m_lastSimTime) / 1000000000.;
    m_lastSimTime = now;

    m_drivebaseSim.update(period);
  }
}
