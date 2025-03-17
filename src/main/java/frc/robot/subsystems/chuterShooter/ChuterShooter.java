package frc.robot.subsystems.chuterShooter;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Tuner;

public class ChuterShooter extends SubsystemBase {
    private final ChuterShooterIO io;
    private final ChuterShooterIOInputsAutoLogged inputs = new ChuterShooterIOInputsAutoLogged();

    final Tuner shooterSpeedTuner = new Tuner("ShooterSpeed", 0.3, true);

    private boolean coralLoading = false;
    private boolean coralReady = false;

    public ChuterShooter(ChuterShooterIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        coralLoading = inputs.coralLoading;
        coralReady = inputs.coralReady;

        if (DriverStation.isDisabled()) {
            io.setShooterSpeed(0);
        }
    }

    public void startShooting() {
        io.setShooterSpeed(shooterSpeedTuner.get());
    }

    public void setShooterMotor(double speed) {
        io.setShooterSpeed(speed);
    }

    public void stopShooting() {
        io.stopShooting();
    }

    public boolean isCoralLoading() {
        return coralLoading;
    }

    public boolean isCoralReady() {
        return coralReady;
    }

    public void scheduleShoot(double speed, double delaySeconds) {
        io.scheduleShoot(speed, delaySeconds);
    }

    public Command loadCoralChute() {
        return Commands.sequence(
                Commands.waitUntil(() -> this.isCoralLoading()),
                Commands.run(() -> this.setShooterMotor(0.1), this),
                Commands.waitUntil(() -> this.isCoralReady())).finallyDo(() -> this.setShooterMotor(0))
                .withTimeout(Time.ofBaseUnits(15, Seconds));
    }
}
