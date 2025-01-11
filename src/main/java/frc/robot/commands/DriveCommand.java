// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.Gamepads.JOY_ROTATE_SCALE;
import static frc.robot.Constants.Gamepads.JOY_SPEED_SCALE;

//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.DrivebaseSubsystem;

public class DriveCommand extends Command {

  private final DrivebaseSubsystem m_DrivebaseSubsystem;
  private DoubleSupplier m_left_y;
  private DoubleSupplier m_right_x;

  public DriveCommand(DoubleSupplier left_y, DoubleSupplier right_x, DrivebaseSubsystem subsystem) {
    m_left_y = left_y;
    m_right_x = right_x;
    m_DrivebaseSubsystem = subsystem;
    addRequirements(m_DrivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed;
    double rotation;
    double rotate_clockwise = m_right_x.getAsDouble() * JOY_ROTATE_SCALE;
    double speed_forward = m_left_y.getAsDouble() * JOY_SPEED_SCALE;

    if (Math.abs(speed_forward) < 0.01) {
      speed = 0;
    } else {
      speed = speed_forward;
    }

    if (Math.abs(rotate_clockwise) < 0.01) {
      rotation = 0;
    } else {
      rotation = rotate_clockwise;
    }
    m_DrivebaseSubsystem.set(speed, rotation);

    // SmartDashboard.putNumber("Y", m_left_y.getAsDouble());
    // SmartDashboard.putNumber("X", m_right_x.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DrivebaseSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
