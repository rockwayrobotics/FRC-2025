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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import frc.robot.Constants;

/**
 * Lightweight drive sim that does not simulate motors
 */
public class DriveIOSimLite implements DriveIO {
  private static final double MAX_VOLTAGE = 12.0;
  private final DifferentialDrive differentialDrive;

  private DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(DCMotor.getNEO(2),
      Constants.Drive.WHEEL_GEAR_RATIO,
      // FIXME: These values are defaults from
      // https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/drivesim-tutorial/drivetrain-model.html
      // and really should be measured.
      Constants.PathPlanner.ROBOT_MOI,
      Constants.PathPlanner.ROBOT_MASS_KG,
      Constants.Drive.WHEEL_RADIUS_METERS,
      Constants.Drive.TRACK_WIDTH_METERS,
      // TODO: Add noise to the simulation here as standard deviation values for
      // noise:
      // x, y in m
      // heading in rad
      // l/r velocity m/s
      // l/r position in m
      VecBuilder.fill(0, 0, 0, 0, 0, 0, 0));

  private double leftAppliedVolts = 0.0;
  private double rightAppliedVolts = 0.0;

  // FIXME: The simulator uses a PID controller here. Why?
  private boolean closedLoop = false;
  private static final double simKp = 0.05;
  private static final double simKd = 0.0;
  private PIDController leftPID = new PIDController(simKp, 0.0, simKd);
  private PIDController rightPID = new PIDController(simKp, 0.0, simKd);

  private double leftFFVolts = 0.0;
  private double rightFFVolts = 0.0;

  public DriveIOSimLite() {
    differentialDrive = new DifferentialDrive((double leftSpeed) -> {
      leftAppliedVolts = leftSpeed * MAX_VOLTAGE;
    }, (double rightSpeed) -> {
      rightAppliedVolts = rightSpeed * MAX_VOLTAGE;
    });
    differentialDrive.setSafetyEnabled(false);
  }

  @Override
  public DifferentialDrive getDifferentialDrive() {
    return differentialDrive;
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    if (closedLoop) {
      leftAppliedVolts = leftFFVolts
          + leftPID.calculate(sim.getLeftVelocityMetersPerSecond());
      rightAppliedVolts = rightFFVolts
          + rightPID.calculate(sim.getRightVelocityMetersPerSecond());
    }

    // Update simulation state
    sim.setInputs(
        MathUtil.clamp(leftAppliedVolts, -MAX_VOLTAGE, MAX_VOLTAGE),
        MathUtil.clamp(rightAppliedVolts, -MAX_VOLTAGE, MAX_VOLTAGE));
    sim.update(0.02);

    inputs.leftPositionMeters = sim.getLeftPositionMeters();
    inputs.leftVelocityMetersPerSec = sim.getLeftVelocityMetersPerSecond();
    inputs.leftAppliedVolts = leftAppliedVolts;
    inputs.leftCurrentAmps = new double[] { sim.getLeftCurrentDrawAmps() };

    inputs.rightPositionMeters = sim.getRightPositionMeters();
    inputs.rightVelocityMetersPerSec = sim.getRightVelocityMetersPerSecond();
    inputs.rightAppliedVolts = rightAppliedVolts;
    inputs.rightCurrentAmps = new double[] { sim.getRightCurrentDrawAmps() };
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    closedLoop = false;
    leftAppliedVolts = leftVolts;
    rightAppliedVolts = rightVolts;
  }

  @Override
  public void setVelocity(
      double leftMetersPerSec, double rightMetersPerSec, double leftFFVolts, double rightFFVolts) {
    closedLoop = true;
    this.leftFFVolts = leftFFVolts;
    this.rightFFVolts = rightFFVolts;
    leftPID.setSetpoint(leftMetersPerSec);
    rightPID.setSetpoint(rightMetersPerSec);
  }
}