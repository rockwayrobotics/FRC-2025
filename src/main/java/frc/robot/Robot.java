// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.m_angler.angleSetpointWidget.setDouble(0);
    m_robotContainer.onDisable();

    // Expect this to print zeros at first, but do it anyway as a sanity-check.
    // We should actually see the output from here twice, once at startup
    // and once after the game is over when the robot should (in theory)
    // be sent back to the disabled state.
    m_robotContainer.m_drivebase.reportFailures("disabled"); // print encoder reset fail counts
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_robotContainer.m_angler.resetAngleEncoder();
    // m_robotContainer.m_drivebase.resetPose(new Pose2d(2,7, new Rotation2d(0)));
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // m_robotContainer.m_angler.angleSetpointWidget.setDouble(Constants.Angler.AUTO_SPEAKER_SETPOINT);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.m_drivebase.reportFailures("auto"); // print encoder reset fail counts
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.onTeleopInit();
    m_robotContainer.m_angler.angleSetpointWidget.setDouble(Constants.Angler.SPEAKER_SETPOINT);

    // m_robotContainer.m_drivebaseSubsystem.setDrivebaseIdle(DrivebaseSubsystem.IdleMode.kCoast);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // if (!m_robotContainer.m_intake.intakeLoad &&
    // m_robotContainer.m_intake.isNoteLoaded()){
    // m_robotContainer.noteStage();
    // }
  }

  @Override
  public void teleopExit() {
    m_robotContainer.m_drivebase.reportFailures("teleop"); // print encoder reset fail counts
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
    m_robotContainer.onSimulationInit();
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
