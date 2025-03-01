package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.ScoringState.SensorState;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.ScoreCommands;
import frc.robot.simulation.WorldSimulation;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteIOReal;
import frc.robot.subsystems.chute.ChuteIOSim;
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

public class RobotContainer {
  // Subsystems are listed here
  private final Drive drive;
  private final Superstructure superstructure;
  private final Climp climp;
  // Control devices
  private final CommandXboxController driverController = new CommandXboxController(Constants.Gamepads.DRIVER);
  private final CommandXboxController operatorController = new CommandXboxController(Constants.Gamepads.OPERATOR);

  // Dashboard inputs
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();
  private final ShuffleboardTab dashboard = Shuffleboard.getTab("RobotContainer");
  private final GenericEntry driveScale = dashboard.addPersistent("Drivescale", 1).getEntry();

  // Simulation only
  protected WorldSimulation simulation = null;

  public RobotContainer() {
    if (RobotBase.isReal()) {
      drive = new Drive(new DriveIOSparkMax(), new GyroIONavX());

      Elevator elevator = new Elevator(new ElevatorIOReal());
      Chute chute = new Chute(new ChuteIOSim());
      Grabber grabber = new Grabber(new GrabberIOSim());
      superstructure = new Superstructure(elevator, chute, grabber);
      climp = new Climp(new ClimpIOSim());
    } else {
      simulation = new WorldSimulation();
      drive = simulation.getDrive();
      superstructure = new Superstructure(new Elevator(simulation.getElevator()), new Chute(simulation.getChute()),
          new Grabber(simulation.getGrabber()));
      climp = new Climp(simulation.getClimp());
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
    boolean enabled = false;
    if (enabled) {
      driverController.leftBumper().onTrue(new InstantCommand(() -> drive.setScale(driveScale.getDouble(0.3))));
      driverController.leftBumper().onFalse(new InstantCommand(() -> drive.setScale(1)));

      driverController.a().whileTrue(ScoreCommands.score(drive, superstructure));

      operatorController.povUpLeft().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_LEFT;
      }));
      operatorController.povUpRight().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_RIGHT;
      }));
      operatorController.povDownLeft().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_LEFT;
      }));
      operatorController.povDownRight().onTrue(new InstantCommand(() -> {
        RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_RIGHT;
      }));
      operatorController.povLeft().onTrue(new InstantCommand(() -> {
        var sensorState = RobotTracker.getInstance().getScoringState().sensorState;
        if (sensorState == SensorState.BACK_LEFT || sensorState == SensorState.BACK_RIGHT) {
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_LEFT;
        } else {
          // Default to front
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_LEFT;
        }
      }));
      operatorController.povRight().onTrue(new InstantCommand(() -> {
        var sensorState = RobotTracker.getInstance().getScoringState().sensorState;
        if (sensorState == SensorState.BACK_LEFT || sensorState == SensorState.BACK_RIGHT) {
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_RIGHT;
        } else {
          // Default to front
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.FRONT_RIGHT;
        }
      }));
      operatorController.povDown().onTrue(new InstantCommand(() -> {
        var sensorState = RobotTracker.getInstance().getScoringState().sensorState;
        if (sensorState == SensorState.FRONT_RIGHT || sensorState == SensorState.BACK_RIGHT) {
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_RIGHT;
        } else {
          // Default to left
          RobotTracker.getInstance().getScoringState().sensorState = SensorState.BACK_LEFT;
        }
      }));
      operatorController.povUp().onTrue(new InstantCommand(() -> {
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
            () -> superstructure.elevator.setGoalHeightMeters(0.6),
            superstructure));
    // PoV Up but with different event loop
    driverController.pov(0, 180, testModelButtonLoop)
        .onTrue(Commands.runOnce(
            () -> superstructure.elevator.setGoalHeightMeters(0.4),
            superstructure));

    // FIXME FIXME FIXME: Everything is disabled for now
    boolean enabled = false;
    if (enabled) {
      // PoV Right but with different event loop
      driverController.pov(0, 90, testModelButtonLoop)
          .onTrue(Commands.runOnce(
              () -> superstructure.chute
                  .setPivotGoalRads(-Units.degreesToRadians(1) + superstructure.chute.getPivotGoalRads()),
              superstructure));
      // PoV Left but with different event loop
      driverController.pov(0, 270, testModelButtonLoop)
          .onTrue(Commands.runOnce(
              () -> superstructure.chute
                  .setPivotGoalRads(Units.degreesToRadians(1) + superstructure.chute.getPivotGoalRads()),
              superstructure));
      driverController.a(testModelButtonLoop)
          .whileTrue(Commands.run(() -> superstructure.chute.startShooting(), superstructure)
              .finallyDo(() -> superstructure.chute.stopShooting()));
      driverController.b(testModelButtonLoop)
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
