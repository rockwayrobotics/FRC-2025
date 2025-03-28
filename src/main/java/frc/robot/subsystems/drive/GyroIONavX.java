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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.NavX.AHRS;
import edu.wpi.first.wpilibj.SPI.Port;


/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  protected final AHRS navX = new AHRS(Port.kMXP, (byte) 50);
  private boolean initialYawSet = false;
  private double initialYaw = 0;

  @Override   
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = navX.isConnected();
    if (!initialYawSet && inputs.connected) {
      initialYaw = -navX.getAngle();
      initialYawSet = true;
    }
    inputs.yawPosition = Rotation2d.fromDegrees(-navX.getAngle() - initialYaw);
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-navX.getRawGyroZ());
  }

  @Override
  public void zeroGyro() {
    navX.zeroYaw();
    initialYaw = 0;
    initialYawSet = true;
  }

  @Override
  public double getAngle() {
    return navX.getAngle();
  }
}
