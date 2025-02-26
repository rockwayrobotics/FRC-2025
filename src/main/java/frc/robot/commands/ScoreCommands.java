package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.chute.Chute;
import frc.robot.subsystems.drive.Drive;

public class ScoreCommands {
    public static Command score(Drive drive, Elevator elevator, Chute chute) {
        var nt = NetworkTableInstance.getDefault();
        var piState = nt.getIntegerTopic("pi-state").publish();
        var corner = nt.getFloatTopic("corner").subscribe(0);
        AtomicReference<Float> cornerTime = new AtomicReference<Float>(0.0f);
        var cancel = new ParallelRaceGroup();
        var command = Commands.parallel(
            Commands.run(() -> {
                drive.setTankDrive(new ChassisSpeeds(0.45, 0, 0));
            }),
            Commands.runOnce(() -> {
                chute.setPivotGoalRads(Radians.convertFrom(30, Degrees));
            }),
            Commands.sequence(
                Commands.runOnce(() -> piState.set(1)),
                Commands.waitUntil(() -> {
                    var corners = corner.readQueueValues();
                    if (corners.length > 0) {
                        cornerTime.set(corners[corners.length - 1]);
                        return true;
                    }
                    return false;
                }),
                Commands.runOnce(() -> {
                    var leftEncoderDistance = drive.getLeftPositionAtTime(cornerTime.get());
                    if (leftEncoderDistance.isEmpty()) {
                        cancel.addCommands(Commands.runOnce(() -> {}));
                    }

                })
            )
        );
        command.addRequirements(drive, elevator, chute);
        cancel.addCommands(command);
        return cancel;
    }
}
