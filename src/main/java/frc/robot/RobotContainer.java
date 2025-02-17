package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.RobotTracker.ToFSide;
import frc.robot.commands.DockingCommand;
import frc.robot.commands.DriveCommands;
import frc.robot.simulation.WorldSimulation;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.chute.ChuteIOReal;
import frc.robot.subsystems.chute.ChuteIOSim;
import frc.robot.subsystems.climp.Climp;
import frc.robot.subsystems.climp.ClimpIOReal;
import frc.robot.subsystems.climp.ClimpIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveIOSim;
import frc.robot.subsystems.drive.DriveIOSimComplex;
import frc.robot.subsystems.drive.DriveIOSimLite;
import frc.robot.subsystems.drive.DriveIOSparkMax;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIONavX;
import frc.robot.subsystems.drive.GyroIOSim;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIOSim;
import frc.robot.subsystems.elevator.ElevatorIOSparkMax;
import frc.robot.subsystems.grabber.Grabber;
import frc.robot.subsystems.grabber.GrabberIOReal;
import frc.robot.subsystems.grabber.GrabberIOSim;
import frc.robot.subsystems.tof.ToF;
import frc.robot.subsystems.tof.ToFIOPi5;

public class RobotContainer {
  // Subsystems are listed here
  private final Drive drive;
  private final Elevator elevator;
  private final Chute chute;
  private final Grabber grabber;
  private final Climp climp;
  private final ToF tof;

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
      elevator = new Elevator(new ElevatorIOSparkMax());
      chute = new Chute(new ChuteIOReal());
      grabber = new Grabber(new GrabberIOReal());
      climp = new Climp(new ClimpIOReal());
      tof = new ToF(new ToFIOPi5());
    } else {
      simulation = new WorldSimulation();
      drive = simulation.getDrive();
      elevator = new Elevator(simulation.getElevator());
      chute = new Chute(simulation.getChute());
      grabber = new Grabber(simulation.getGrabber());
      climp = new Climp(simulation.getClimp());
      tof = new ToF(simulation.getToF());
    }

    // Set up auto routines
    // Set up SysId routines
    autoChooser.addOption("Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption("Drive SysId (Quasistatic Forward)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Quasistatic Reverse)", drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption("Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption("SysID All (Rev then Fwd)", drive.sysIDRunAll());
    autoChooser.addOption("Set speed = 1", new RunCommand(() -> drive.runClosedLoop(2, 2), drive));
    autoChooser.addOption("Pathplanner", new PathPlannerAuto("straightlong"));

    dashboard.add("Auto Routine", autoChooser).withSize(2, 1).withPosition(8, 0);

    configureBindings();

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

    // driver & operator A              -> Coral.Pivot (swap side)
    // driver & operator B              -> Coral.shoot
    // driver & operator X              -> Elevator.set-L2
    // driver & operator Y              -> Elevator.set-L3
    // driver & operator left trigger   -> Algae.intake
    // driver & operator right trigger  -> Algae.shoot
    // driver & operator D-pad up       -> Coral.shift-up
    // driver & operator D-pad down     -> Coral.shift-down
    // driver & operator D-pad left     -> Algae.up
    // driver & operator D-pad right    -> Algae.down
    
    // driver & operator right bumper held -> landing sequence
    driverController.rightBumper().whileTrue(new DockingCommand(ToFSide.LEFT, drive, chute));

    // left bumper -> set drive scale to 0.3 when held
    driverController.leftBumper().onTrue(new InstantCommand(() -> drive.setScale(driveScale.getDouble(0.3))));
    driverController.leftBumper().onFalse(new InstantCommand(() -> drive.setScale(1)));

    // operator left-stick up/down -> Elevator up/down

    // operator B -> Coral shoot
    operatorController.b().onTrue(Commands.runOnce(() -> chute.shoot(), chute));
    // operator X -> Elevator set L2
    operatorController.x().onTrue(Commands.runOnce(() -> elevator.setGoalHeightMeters(1.106), elevator));
    // operator Y -> Elevator set L3
    operatorController.y().onTrue(Commands.runOnce(() -> elevator.setGoalHeightMeters(1.51), elevator));
    // operator D-pad up -> Coral angle up
    operatorController.povUp().onTrue(Commands
        .runOnce(() -> chute.setPivotGoalRads(chute.getPivotGoalRads() - Radians.convertFrom(2.5, Degrees)), chute));
    // operator D-pad down -> Coral angle down
    operatorController.povDown().onTrue(Commands
        .runOnce(() -> chute.setPivotGoalRads(chute.getPivotGoalRads() + Radians.convertFrom(2.5, Degrees)), chute));

    drive.setDefaultCommand(DriveCommands.defaultDrive(driverController::getLeftY, driverController::getRightX, drive));

    if (RobotBase.isSimulation()) {
      // Add coral with driver right bumper
      driverController.rightBumper().onTrue(Commands.runOnce(() -> simulation.addCoral()));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public WorldSimulation getWorldSimulation() {
    return simulation;
  }
}
