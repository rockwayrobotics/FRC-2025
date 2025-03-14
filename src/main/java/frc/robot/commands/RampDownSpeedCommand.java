package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.RampDownSpeed;

public class RampDownSpeedCommand extends Command {
    Drive drive;
    double targetDistanceMeters;
    double maxDeceleration;
    double initialPositionMeters;
    RampDownSpeed ramp;

    public RampDownSpeedCommand(Drive drive, double targetDistanceMeters, double maxDeceleration) {
        this.drive = drive;
        this.targetDistanceMeters = targetDistanceMeters;
        this.maxDeceleration = maxDeceleration;
    }

    private double getPosition() {
        return (drive.getLeftPositionMeters() + drive.getRightPositionMeters()) / 2;
    }

    private double getVelocity() {
        return (drive.getLeftVelocityMetersPerSec() + drive.getRightVelocityMetersPerSec()) / 2;
    }

    private double remainingDistance() {
      return targetDistanceMeters - (getPosition() - initialPositionMeters);
    }

    @Override
    public void initialize() {
        this.ramp = new RampDownSpeed(getVelocity(), targetDistanceMeters, maxDeceleration);
        this.initialPositionMeters = getPosition();
    }

    @Override
    public void execute() {
        var speed = ramp.calculateSpeed(remainingDistance());
        drive.setTankDrive(new ChassisSpeeds(speed, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return remainingDistance() < 0.05;
    }

    @Override
    public void end(boolean cancelled) {
        drive.stop();
    }
}
