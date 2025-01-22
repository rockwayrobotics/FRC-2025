package frc.robot.commands;

import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec

  private DriveCommands() {
  }

  public static Command defaultDrive(DoubleSupplier left_y, DoubleSupplier right_x, Drive drive) {
    return Commands.run(
        () -> {
          double speed;
          double rotation;
          double rotate_clockwise = right_x.getAsDouble() * Constants.Gamepads.JOY_ROTATE_SCALE;
          double speed_forward = left_y.getAsDouble() * Constants.Gamepads.JOY_SPEED_SCALE;

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
          drive.set(speed, rotation);
        }, drive).finallyDo((boolean interrupted) -> {
          drive.stop();
        });
  }

  /**
   * Standard joystick drive, where X is the forward-backward axis (positive =
   * forward) and Z is the
   * left-right axis (positive = counter-clockwise).
   */
  public static Command arcadeDrive(
      Drive drive, DoubleSupplier xSupplier, DoubleSupplier zSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double x = MathUtil.applyDeadband(xSupplier.getAsDouble(), DEADBAND);
          double z = MathUtil.applyDeadband(zSupplier.getAsDouble(), DEADBAND);

          // Calculate speeds
          var speeds = DifferentialDrive.arcadeDriveIK(x, z, true);

          // Apply output
          drive.runClosedLoop(
              speeds.left * Constants.Drive.MAX_SPEED_MPS, speeds.right * Constants.Drive.MAX_SPEED_MPS);
        },
        drive);
  }

  /** Measures the velocity feedforward constants for the drive. */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
              timer.restart();
            }),

        // Accelerate and gather data
        Commands.run(
            () -> {
              double voltage = timer.get() * FF_RAMP_RATE;
              drive.runOpenLoop(voltage, voltage);
              velocitySamples.add(drive.getCharacterizationVelocity());
              voltageSamples.add(voltage);
            },
            drive)

            // When cancelled, calculate and print results
            .finallyDo(
                () -> {
                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println("********** Drive FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
