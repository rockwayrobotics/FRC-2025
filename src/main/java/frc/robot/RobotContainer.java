// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Gamepads;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveForward;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.LedSubsystem;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Gamepads.DRIVER);
  private final CommandXboxController m_operatorController = new CommandXboxController(Gamepads.OPERATOR);

  public final DrivebaseSubsystem m_drivebase = new DrivebaseSubsystem(Robot.isSimulation());
  public final LedSubsystem m_led = new LedSubsystem();

  SendableChooser<String> m_autoChooser = new SendableChooser<>();

  ShuffleboardTab dashboard = Shuffleboard.getTab("RobotContainer");

  GenericEntry drivescale = dashboard.addPersistent("Drivescale", 1)
      .getEntry();

  /**
   * 
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    m_autoChooser.setDefaultOption("Drive Forward", "driveForward");
    m_autoChooser.addOption("Path Planner Example", "pathPlannerExample");
    m_autoChooser.addOption("Path Planner Straight", "pathPlannerStraight");
    dashboard.add("Auto Routine", m_autoChooser).withSize(2, 1).withPosition(8, 0);

    m_drivebase
        .setDefaultCommand(new DriveCommand(m_driverController::getLeftY, m_driverController::getRightX, m_drivebase));

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

    // left bumper -> set drive scale to 0.3 when held
    m_driverController.leftBumper().onTrue(new InstantCommand(() -> m_drivebase.setScale(drivescale.getDouble(0.3))));
    m_driverController.leftBumper().onFalse(new InstantCommand(() -> m_drivebase.setScale(1)));

  }

  public void onDisable() {
    m_drivebase.disable();
  }

  public void onTeleopInit() {
    m_drivebase.setDrivebaseIdle(IdleMode.kBrake);
    m_led.setMode(Constants.LED.modes.Rainbow);
  }

  public void onSimulationInit() {
    m_drivebase.onSimulationInit();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous

    return switch (m_autoChooser.getSelected()) {
      case "pathPlannerExample" -> new PathPlannerAuto("New Auto");
      case "pathPlannerStraight" -> new PathPlannerAuto("New New Auto");
      case "driveForward" -> new DriveForward(m_drivebase);
      default -> null;
      // FIXME
    };
  }
}
