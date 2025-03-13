package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.Side;
import frc.robot.Constants.AlgaeLevel;
import frc.robot.Constants.CoralLevel;
import frc.robot.ScoringState.SensorState;
import frc.robot.commands.AutoPaths;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.DriveStraight;
import frc.robot.commands.ScoreCommandsOnlyDrive;
import frc.robot.simulation.WorldSimulation;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteIOReal;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.chuterShooter.ChuterShooter;
import frc.robot.subsystems.chuterShooter.ChuterShooterIOReal;
import frc.robot.subsystems.climp.Climp;
import frc.robot.subsystems.climp.ClimpIOReal;
import frc.robot.subsystems.climp.ClimpIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSimComplex;
import frc.robot.subsystems.drive.DriveIOSimLite;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.elevator.ElevatorIOReal;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberIOReal;
import frc.robot.subsystems.grabber.GrabberIOSim;
import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.util.Sensors;
import frc.robot.util.Tuner;

public class RobotContainer {
  // Subsystems are listed here
  private final Drive drive;
  private final Superstructure superstructure;
  private final Climp climp;
  private final ChuterShooter chuterShooter;

  // Control devices
  private final CommandXboxController driverController = new CommandXboxController(Constants.Gamepads.DRIVER);
  private final GenericHID operator1Controller = new GenericHID(Constants.Gamepads.OPERATOR_1);
  private final GenericHID operator2Controller = new GenericHID(Constants.Gamepads.OPERATOR_2);
  // FIXME this should be the same as operator controller probably
  private final CommandXboxController testController = new CommandXboxController(Constants.Gamepads.TEST);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final ShuffleboardTab dashboard = Shuffleboard.getTab("RobotContainer");
  private final GenericEntry driveScale = dashboard.addPersistent("Drivescale", 1).getEntry();
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final Tuner rotationTuner = new Tuner("rotationTuner", 0.7, true);

  // Set up a camera publisher so I can publish value of camera to NetworkTables
  private final NetworkTableEntry cameraPublisher = nt.getEntry("/CameraPublisher/PiCam/selected");

  // Simulation only
  protected WorldSimulation simulation = null;

  public RobotContainer() {
    if (RobotBase.isReal()) {
      drive = new Drive(new DriveIOSparkMax(), new GyroIONavX());

      Elevator elevator = new Elevator(new ElevatorIOReal());
      Chute chute = new Chute(new ChuteIOReal());
      Grabber grabber = new Grabber(new GrabberIOReal());
      superstructure = new Superstructure(elevator, chute, grabber);
      climp = new Climp(new ClimpIOReal());
      chuterShooter = new ChuterShooter(new ChuterShooterIOReal());
    } else {
      simulation = new WorldSimulation();
      drive = simulation.getDrive();
      superstructure = new Superstructure(new Elevator(simulation.getElevator()), new Chute(simulation.getChute()),
          new Grabber(simulation.getGrabber()));
      climp = new Climp(simulation.getClimp());
      chuterShooter = new ChuterShooter(simulation.getChuterShooter());
    }

    // Set up auto routines
    // Set up SysId routines
    autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("SysID All (Rev then Fwd)", drive.sysIDRunAll());
    // Remove Pathplanner paths for now
    // autoChooser.addOption("Pathplanner", new PathPlannerAuto("straightlong"));
    autoChooser.addOption("Forward", DriveCommands.driveForward(drive));
    autoChooser.addOption("Auto1", DriveCommands.auto1(drive));
    autoChooser.addOption("Auto2", DriveCommands.auto2(drive));
    autoChooser.addOption("Auto3", DriveCommands.auto3(drive));
    autoChooser.addOption("Auto4", DriveCommands.auto4(drive));
    autoChooser.addOption("ToReef", DriveCommands.toReef(drive));
    autoChooser.addOption("FromReef", DriveCommands.fromReef(drive));

    // new autos that are UNTESTED
    autoChooser.addOption("justDrive", AutoPaths.justMove(drive, superstructure));
    autoChooser.addOption("pushRookiesV2", AutoPaths.pushRookies(drive, superstructure));
    autoChooser.addOption("midFarRightL2", AutoPaths.midFarRightL2(drive, superstructure, chuterShooter));
    autoChooser.addOption("rightNearCenterL2", AutoPaths.rightNearCenterL2(drive, superstructure, chuterShooter));
    autoChooser.addOption("leftNearCenterL2", AutoPaths.leftNearCenterL2(drive, superstructure, chuterShooter));
    autoChooser.addOption("rightNearRightTrough", AutoPaths.rightNearRightTrough(drive, superstructure, chuterShooter));
    autoChooser.addOption("leftNearLeftTrough", AutoPaths.leftNearLeftTrough(drive, superstructure, chuterShooter));
    autoChooser.addOption("rightNearCenterTrough", AutoPaths.rightNearCenterTrough(drive, superstructure, chuterShooter));
    autoChooser.addOption("leftNearCenterTrough", AutoPaths.leftNearCenterTrough(drive, superstructure, chuterShooter));
    autoChooser.addOption("rightTestTrough", AutoPaths.rightTestTrough(drive, superstructure, chuterShooter));
    autoChooser.addOption("leftTestTrough", AutoPaths.leftTestTrough(drive, superstructure, chuterShooter));

    dashboard.add("Auto Routine", autoChooser).withSize(2, 1).withPosition(8, 0);

    configureBindings();

  }

  public WorldSimulation getWorldSimulation() {
    return simulation;
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // driver & operator A -> Coral.Pivot (swap side)
    // driver & operator B -> Coral.shoot
    // driver & operator X -> Elevator.set-L2
    // driver & operator Y -> Elevator.set-L3
    // driver & operator left trigger -> Algae.intake
    // driver & operator right trigger -> Algae.shoot
    // driver & operator D-pad up -> Coral.shift-up
    // driver & operator D-pad down -> Coral.shift-down
    // driver & operator D-pad left -> Algae.up
    // driver & operator D-pad right -> Algae.down

    // driver & operator right bumper held -> landing sequence

    // left bumper -> set drive scale to 0.3 when held

    // FIXME FIXME FIXME: Everything is disabled for now
    boolean enabled = true;
    if (enabled) {
      driverController.leftBumper().onTrue(new InstantCommand(() -> drive.setScale(driveScale.getDouble(0.3))));
      driverController.leftBumper().onFalse(new InstantCommand(() -> drive.setScale(0.7)));

      driverController.rightBumper().onTrue(new InstantCommand(() -> drive.setScale(1)));
      driverController.rightBumper().onFalse(new InstantCommand(() -> drive.setScale(0.7)));

      // driverController.leftTrigger().onTrue(new InstantCommand(() -> drive.setRotationScale(rotationTuner.get())));
      // driverController.leftTrigger().onFalse(new InstantCommand(() -> drive.setRotationScale(0.76)));

      driverController.leftTrigger().onTrue(new InstantCommand(() -> drive.setScale(0.1)));
      driverController.leftTrigger().onFalse(new InstantCommand(() -> drive.setScale(0.7)));

      driverController.rightTrigger().onTrue(new InstantCommand(() -> drive.setRotationScale(1)));
      driverController.rightTrigger().onFalse(new InstantCommand(() -> drive.setRotationScale(0.76)));

      // driverController.rightTrigger()
      //     .whileTrue(new RepeatCommand(new InstantCommand(() -> drive.set(0.175, driverController.getRightX()))));
      // driverController.rightBumper().onFalse(new InstantCommand(() -> drive.set(0, 0)));

      driverController.a().whileTrue(ScoreCommandsOnlyDrive.score(drive, superstructure.chute, Constants.ReefBar.NEAR, chuterShooter));
      driverController.b().whileTrue(ScoreCommandsOnlyDrive.score(drive, superstructure.chute, Constants.ReefBar.FAR, chuterShooter));
      driverController.x().whileTrue(new DriveStraight(0.45, drive));

      // driverController.rightBumper()
      //     .whileTrue(Commands.run(() -> superstructure.chute.startShooting(), superstructure));
      // driverController.rightBumper().onFalse(Commands.runOnce(() -> superstructure.chute.stopShooting()));

      testController.povUpLeft().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_LEFT;
      }));
      testController.povUpRight().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_RIGHT;
      }));
      testController.povDownLeft().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_LEFT;
      }));
      testController.povDownRight().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_RIGHT;
      }));
      testController.povLeft().onTrue(new InstantCommand(() -> {
        var sensorState = RobotTracker.getInstance().getScoringState().sensorState;
        if (sensorState == SensorState.BACK_LEFT || sensorState == SensorState.BACK_RIGHT) {
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_LEFT;
        } else {
          // Default to front
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_LEFT;
        }
      }));
      testController.povRight().onTrue(new InstantCommand(() -> {
        var sensorState = RobotTracker.getInstance().getScoringState().sensorState;
        if (sensorState == SensorState.BACK_LEFT || sensorState == SensorState.BACK_RIGHT) {
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_RIGHT;
        } else {
          // Default to front
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_RIGHT;
        }
      }));
      testController.povDown().onTrue(new InstantCommand(() -> {
        var sensorState = RobotTracker.getInstance().getScoringState().sensorState;
        if (sensorState == SensorState.FRONT_RIGHT || sensorState == SensorState.BACK_RIGHT) {
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_RIGHT;
        } else {
          // Default to left
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_LEFT;
        }
      }));
      testController.povUp().onTrue(new InstantCommand(() -> {
        var sensorState = RobotTracker.getInstance().getScoringState().sensorState;
        if (sensorState == SensorState.FRONT_RIGHT || sensorState == SensorState.BACK_RIGHT) {
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_RIGHT;
        } else {
          // Default to left
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_LEFT;
        }
      }));

      drive.setDefaultCommand(
          DriveCommands.defaultDrive(driverController::getLeftY, driverController::getRightX, drive));
    }

    boolean stubBindings = true;
    if (stubBindings) {
      new JoystickButton(operator2Controller, 1).whileTrue(Commands.run(() -> {
        superstructure.setElevatorGoalHeightMillimeters(superstructure.elevator.getHeightMillimeters() + 60);
      }, superstructure));

      new JoystickButton(operator2Controller, 2).whileTrue(Commands.run(() -> {
        superstructure.setElevatorGoalHeightMillimeters(superstructure.elevator.getHeightMillimeters() - 60);
      }, superstructure));

      new JoystickButton(operator2Controller, 3).whileTrue(Commands.run(() -> {
        if (superstructure.getElevatorHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
          superstructure.setChutePivotGoalRads(superstructure.chute.getPivotAngleRads() + 0.4);
        }
      }, superstructure)).onFalse(Commands.run(() -> {
        superstructure.chute.setPivotGoalRads(superstructure.chute.getPivotAngleRads());
      }, superstructure));

      new JoystickButton(operator2Controller, 8).whileTrue(Commands.run(() -> {
        if (superstructure.getElevatorHeightMillimeters() > Constants.Chute.CHUTE_MINUMUM_ELEVATOR_HEIGHT_MM) {
          superstructure.setChutePivotGoalRads(superstructure.chute.getPivotAngleRads() - 0.4);
        }
      }, superstructure)).onFalse(Commands.run(() -> {
        superstructure.chute.setPivotGoalRads(superstructure.chute.getPivotAngleRads());
      }, superstructure));

      // new JoystickButton(operator2Controller, 4).whileTrue(Commands.run(() -> {
      // climp.setClimpGoalRads(climp.getClimpAngleRads() + 0.01);
      // }, climp)).onFalse(Commands.run(() -> {
      // climp.setClimpGoalRads(climp.getClimpAngleRads());
      // }, climp));

      // new JoystickButton(operator2Controller, 6).whileTrue(Commands.run(() -> {
      // climp.setClimpGoalRads(climp.getClimpAngleRads() - 0.01);
      // }, climp)).onFalse(Commands.run(() -> {
      // climp.setClimpGoalRads(climp.getClimpAngleRads());
      // }, climp));

      new JoystickButton(operator2Controller, 4).whileTrue(Commands.run(() -> {
        climp.setNormalizedSpeed(1);
      }, climp)).onFalse(Commands.run(() -> {
        climp.setNormalizedSpeed(0);
      }, climp));

      new JoystickButton(operator2Controller, 6).whileTrue(Commands.run(() -> {
        climp.setNormalizedSpeed(-1);
      }, climp)).onFalse(Commands.run(() -> {
        climp.setNormalizedSpeed(0);
      }, climp));

      new JoystickButton(operator2Controller, 5).whileTrue(Commands.run(() -> {
        chuterShooter.startShooting();
      }, chuterShooter)).onFalse(Commands.run(() -> {
        chuterShooter.setShooterMotor(0);
      }, chuterShooter));

      new JoystickButton(operator2Controller, 7).whileTrue(Commands.run(() -> {
        chuterShooter.setShooterMotor(-0.1); // intake
      }, chuterShooter)).onFalse(Commands.run(() -> {
        chuterShooter.stopShooting();
      }, chuterShooter));

      new POVButton(operator2Controller, 180).whileTrue(Commands.run(() -> {
        superstructure.setWristGoalRads(superstructure.grabber.getCurrentRads() + 0.5);
      }, superstructure)).onFalse(Commands.run(() -> {
        superstructure.setWristGoalRads(superstructure.grabber.getCurrentRads());
      }, superstructure));

      new POVButton(operator2Controller, 270).whileTrue(Commands.run(() -> {
        superstructure.setWristGoalRads(superstructure.grabber.getCurrentRads() - 0.5);
      }, superstructure)).onFalse(Commands.run(() -> {
        superstructure.setWristGoalRads(superstructure.grabber.getCurrentRads());
      }, superstructure));

      new POVButton(operator2Controller, 0).whileTrue(Commands.run(() -> {
        superstructure.setGrabberMotor(1);
      }, superstructure)).onFalse(Commands.run(() -> {
        superstructure.setGrabberMotor(0);
      }, superstructure));

      new POVButton(operator2Controller, 90).whileTrue(Commands.run(() -> {
        superstructure.setGrabberMotor(-1);
      }, superstructure)).onFalse(Commands.run(() -> {
        superstructure.setGrabberMotor(0);
      }, superstructure));

      // FIXME maybe; cam buttons untestes
      new JoystickButton(operator2Controller, 13).onTrue(Commands.runOnce(() -> {
        cameraPublisher.setString("fore");
      }));

      new JoystickButton(operator2Controller, 14).onTrue(Commands.runOnce(() -> {
        cameraPublisher.setString("fore");
      }));

      new JoystickButton(operator2Controller, 11).onTrue(Commands.runOnce(() -> {
        cameraPublisher.setString("aft");
      }));

      new JoystickButton(operator2Controller, 12).onTrue(Commands.runOnce(() -> {
        cameraPublisher.setString("aft");
      }));

      new JoystickButton(operator2Controller, 9).onTrue(Commands.runOnce(() -> {
        cameraPublisher.setString("auto");
      }));

      new JoystickButton(operator2Controller, 10).onTrue(Commands.runOnce(() -> {
        cameraPublisher.setString("auto");
      }));

      new JoystickButton(operator1Controller, 9).onTrue(Commands.runOnce(() -> {
        superstructure.home();
      }, superstructure));

      new JoystickButton(operator1Controller, 10).onTrue(Commands.runOnce(() -> {
        System.out.println("Overriding homing Chute");
        Sensors.getInstance().overrideChuteHomeSwitch = true;
      }, superstructure));

      new JoystickButton(operator1Controller, 14).onTrue(Commands.runOnce(() -> {
        superstructure.foldForClimp();
      }, superstructure));

      new JoystickButton(operator1Controller, 2).whileTrue(Commands.run(() -> {
        climp.setNormalizedSpeed(1);
      }, climp)).onFalse(Commands.run(() -> {
        climp.setNormalizedSpeed(0);
      }, climp));

      new JoystickButton(operator1Controller, 1).whileTrue(Commands.run(() -> {
        climp.setNormalizedSpeed(-1);
      }, climp)).onFalse(Commands.run(() -> {
        climp.setNormalizedSpeed(0);
      }, climp));

      new JoystickButton(operator1Controller, 4).onTrue(Commands.runOnce(() -> {
        superstructure.gotoSetpoint(CoralLevel.L3, Side.RIGHT);
      }, superstructure));
      new JoystickButton(operator1Controller, 6).onTrue(Commands.runOnce(() -> {
        superstructure.gotoSetpoint(CoralLevel.L3, Side.LEFT);
      }, superstructure));
      new JoystickButton(operator1Controller, 3).onTrue(Commands.runOnce(() -> {
        superstructure.gotoSetpoint(CoralLevel.L2, Side.RIGHT);
      }, superstructure));
      new JoystickButton(operator1Controller, 8).onTrue(Commands.runOnce(() -> {
        superstructure.gotoSetpoint(CoralLevel.L2, Side.LEFT);
      }, superstructure));
      new JoystickButton(operator1Controller, 5).onTrue(Commands.runOnce(() -> {
        superstructure.gotoSetpoint(CoralLevel.L1, Side.LEFT);
      }, superstructure));
      new JoystickButton(operator1Controller, 7).onTrue(Commands.runOnce(() -> {
        superstructure.gotoSetpoint(CoralLevel.L1, Side.RIGHT);
      }, superstructure));

      new JoystickButton(operator1Controller, 12).onTrue(Commands.sequence(
        new ProxyCommand(Commands.runOnce(() -> superstructure.gotoSetpoint(CoralLevel.Intake, Side.LEFT), superstructure)),
        new ProxyCommand(chuterShooter.loadCoralChute())
      ));

      new JoystickButton(operator1Controller, 11).onTrue(Commands.sequence(
        new ProxyCommand(Commands.runOnce(() -> superstructure.gotoSetpoint(CoralLevel.Intake, Side.RIGHT), superstructure)),
        new ProxyCommand(chuterShooter.loadCoralChute())
      ));

      // Algae setpoints
      // FIXME: spin wheels until algae sensor detects algae
      new POVButton(operator1Controller, 270).onTrue(Commands.runOnce(() -> {
        superstructure.gotoAlgaeSetpoint(AlgaeLevel.L3);
      }, superstructure));
      new POVButton(operator1Controller, 180).onTrue(Commands.runOnce(() -> {
        superstructure.gotoAlgaeSetpoint(AlgaeLevel.L2);
      }, superstructure));
      new POVButton(operator1Controller, 90).onTrue(Commands.runOnce(() -> {
        superstructure.gotoAlgaeSetpoint(AlgaeLevel.Floor);
      }, superstructure));
      new JoystickButton(operator1Controller, 13).onTrue(Commands.runOnce(() -> {
        superstructure.gotoAlgaeSetpoint(AlgaeLevel.Score);
      }, superstructure));

      new POVButton(operator1Controller, 0).whileTrue(Commands.run(() -> {
        superstructure.setGrabberMotor(1);
      }, superstructure)).onFalse(Commands.run(() -> {
        superstructure.setGrabberMotor(0);
      }, superstructure));
    }
  }

  public void setupTestBindings() {
    var testModelButtonLoop = new EventLoop();
    CommandScheduler.getInstance().setActiveButtonLoop(testModelButtonLoop);

    // FIXME: Do we want to intentionally limit the motors in some way? This code
    // currently just changes the set points
    // very gradually, but the motors are permitted to go at whatever speed in order
    // to reach the set points.

    // PoV Up but with different event loop
    driverController.pov(0, 0, testModelButtonLoop)
        .onTrue(Commands.runOnce(
            () -> superstructure.setElevatorGoalHeightMillimeters(1000),
            superstructure));
    // PoV Down but with different event loop
    driverController.pov(0, 180, testModelButtonLoop)
        .onTrue(Commands.runOnce(
            () -> superstructure.setElevatorGoalHeightMillimeters(100),
            superstructure));

    driverController.leftBumper(testModelButtonLoop)
        .onTrue(Commands.runOnce(() -> superstructure.grabber.setWristGoalRads(-1), superstructure));
    driverController.rightBumper(testModelButtonLoop)
        .onTrue(Commands.runOnce(() -> superstructure.grabber.setWristGoalRads(0.1), superstructure));

    // PoV Right but with different event loop
    driverController.pov(0, 90, testModelButtonLoop)
        .onTrue(Commands.runOnce(() -> superstructure.chute.setPivotGoalRads(1), superstructure));
    // PoV Left but with different event loop
    driverController.pov(0, 270, testModelButtonLoop)
        .onTrue(Commands.runOnce(() -> superstructure.chute.setPivotGoalRads(-1), superstructure));

    driverController.a(testModelButtonLoop).onTrue(Commands.runOnce(() -> superstructure.home()));
    // FIXME FIXME FIXME: Disable potentially unsafe commands
    boolean enabled = true;
    if (enabled) {
      driverController.x(testModelButtonLoop)
          .whileTrue(Commands.run(() -> chuterShooter.startShooting(), chuterShooter)
              .finallyDo(() -> chuterShooter.stopShooting()));
      driverController.y(testModelButtonLoop)
          .whileTrue(Commands.run(() -> climp.setNormalizedSpeed(0.1)).finallyDo(() -> climp.setNormalizedSpeed(0)));

      // This sets the default command to drive very slowly. Remember to reset this
      // when exiting test mode.
      CommandScheduler.getInstance().cancel(drive.getDefaultCommand());
      drive.setDefaultCommand(DriveCommands.defaultDrive(() -> driverController.getLeftY() * 0.1,
          () -> driverController.getRightX() * 0.1, drive));
    }
  }

  public void resetTestBindings() {
    CommandScheduler.getInstance().setActiveButtonLoop(CommandScheduler.getInstance().getDefaultButtonLoop());
    CommandScheduler.getInstance().cancel(drive.getDefaultCommand());
    drive.setDefaultCommand(DriveCommands.defaultDrive(driverController::getLeftY, driverController::getRightX, drive));
  }

  public void setDriveBrakeMode(boolean brake) {
    drive.setBrakeMode(brake);
    superstructure.chute.setBrakeMode(brake);
    superstructure.grabber.setBrakeMode(brake);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public void fullRobotStayStill() {
    drive.stayStill();
    superstructure.stayStill();
    climp.stayStill();
    chuterShooter.stopShooting();
  }

  public void home() {
    superstructure.home();
  }
}
