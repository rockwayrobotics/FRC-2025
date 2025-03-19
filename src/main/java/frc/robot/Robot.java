// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLogOutputManager;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.Sensors;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  // A command that will set the drivebase to coast mode after a delay.
  // We start the command and store it here when the robot is disabled.
  // We store the command here so we can cancel it if we re-enable the robot.
  protected Command enterCoastModeCommand = null;

  public Robot() {
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);

    if (isReal()) {
      // If the USB stick doesn't exist, then it will still try to write to /U/logs, which may not work.
      Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
      Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
      new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    } else {
      // This is for simulation
      Logger.addDataReceiver(new NT4Publisher());

      // This is for replay!
      // setUseTiming(false); // Run as fast as possible
      // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
      // AdvantageScope (or prompt the user)
      // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
      // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
      // "_sim"))); // Save outputs to a new log
    }

    AutoLogOutputManager.addObject(RobotTracker.getInstance());

    Logger.start();

  }

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
    // We have no USB camera on the Rio
    // CameraServer.startAutomaticCapture();
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
    // Switch thread to high priority to improve loop timing.
    Threads.setCurrentThreadPriority(true, 99);

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // Log updates to RobotTracker in all modes
    RobotTracker.getInstance().periodic();

    // Push updates of sensor state to NetworkTables no matter what
    Sensors.getInstance().updateNT();

    // Back to normal priority
    Threads.setCurrentThreadPriority(false, 10);

    m_robotContainer.sendTimeToPiPeriodic();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    enterCoastModeAfterSeconds(2);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
    enterBrakeMode();
    m_robotContainer.fullRobotStayStill();
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.home();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
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
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.setupTestBindings();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
    m_robotContainer.resetTestBindings();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    m_robotContainer.getWorldSimulation().simulationPeriodic();
  }

  private void enterCoastModeAfterSeconds(double seconds) {
    enterCoastModeCommand = Commands
        .sequence(Commands.waitSeconds(seconds), Commands.runOnce(() -> m_robotContainer.setDriveBrakeMode(false)))
        .ignoringDisable(true);
    enterCoastModeCommand.schedule();
  }

  private void enterBrakeMode() {
    if (enterCoastModeCommand != null) {
      enterCoastModeCommand.cancel();
      enterCoastModeCommand = null;
    }
    m_robotContainer.setDriveBrakeMode(true);
  }
}
