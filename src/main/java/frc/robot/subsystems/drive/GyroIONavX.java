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

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

/** IO implementation for NavX. */
public class GyroIONavX implements GyroIO {
  protected final AHRS navX = new AHRS(NavXComType.kMXP_SPI);
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
}
