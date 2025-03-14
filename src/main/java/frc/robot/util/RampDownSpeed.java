package frc.robot.util;

public class RampDownSpeed {
    private double targetAcceleration;
    private double initialVelocity;
    private double targetDistance;

    public RampDownSpeed(double currentVelocity, double targetDistance, double maxDeceleration) {
        // solve for acceleration with vf^2 = vi^2 + 2aΔx, vf = 0
        var accel = currentVelocity * currentVelocity / (2 * targetDistance);
        this.targetAcceleration = -Math.min(accel, maxDeceleration);
        this.initialVelocity = currentVelocity;
        this.targetDistance = targetDistance;
    }

    public double calculateSpeed(double remainingDistance) {
        if (remainingDistance < 0) {
            return 0;
        }
        // solve for the speed we should be at, with vf^2 = vi^2 + 2aΔx
        var deltaX = this.targetDistance - remainingDistance;
        return Math.sqrt(this.initialVelocity * this.initialVelocity + 2 * this.targetAcceleration * deltaX);
    }
}
