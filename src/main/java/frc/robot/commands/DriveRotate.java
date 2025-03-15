package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

public class DriveRotate extends Command {

    private Drive drive;
    private double angle;
    private double currentAngle; 
    private double direction;

    public DriveRotate(Drive drive, double angle) {
        this.drive = drive;
        this.angle = angle;
        addRequirements(drive);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        currentAngle = drive.getGyroAngle();
        direction = Math.signum(angle - currentAngle);

        System.out.println("move to angle: " + angle);
    }
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        currentAngle = drive.getGyroAngle();
        drive.setTankDrive(new ChassisSpeeds(0,0, 0.5 * direction));
    }
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        drive.stop();
        System.out.println("end, moved to " + drive.getGyroAngle());
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (angle - currentAngle) * direction < 0; 
    }
}