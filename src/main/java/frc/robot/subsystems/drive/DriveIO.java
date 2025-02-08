// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public interface DriveIO {

  @AutoLog
  public static class DriveIOInputs {
    public double leftPositionMeters = 0.0;
    public double leftVelocityMetersPerSec = 0.0;
    public double leftAppliedVolts = 0.0;
    public double[] leftCurrentAmps = new double[] {};

    public double rightPositionMeters = 0.0;
    public double rightVelocityMetersPerSec = 0.0;
    public double rightAppliedVolts = 0.0;
    public double[] rightCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(DriveIOInputs inputs) {
  }

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double leftVolts, double rightVolts) {
  }

  /** Run closed loop at the specified velocity. */
  public default void setVelocity(
      double leftMetersPerSec, double rightMetersPerSec, double leftFFVolts, double rightFFVolts) {
  }

  public default DifferentialDrive getDifferentialDrive() {
    return new DifferentialDrive((double leftOutput) -> {}, (double rightOutput) -> {});
  }
}