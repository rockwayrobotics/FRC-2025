package frc.robot.util;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

public class TrajectoryUtils {
  public static final double FIELD_X_DIMENSION_METERS = 17.548;
  public static final double FIELD_Y_DIMENSION_METERS = 8.05;

  public static Trajectory flipTrajectory(Trajectory trajectory) {
    var states = trajectory.getStates();
    var firstState = states.get(0);
    var firstPose = firstState.poseMeters;
    var newFirstPose = new Pose2d(FIELD_X_DIMENSION_METERS - firstPose.getX(), firstPose.getY(),
        Rotation2d.k180deg.minus(firstPose.getRotation()));
    var newStates = new ArrayList<Trajectory.State>();
    newStates.add(
        new Trajectory.State(
            firstState.timeSeconds,
            firstState.velocityMetersPerSecond,
            firstState.accelerationMetersPerSecondSq,
            newFirstPose,
            firstState.curvatureRadPerMeter));

    for (int i = 1; i < states.size(); i++) {
      var state = states.get(i);
      var pose = state.poseMeters;
      var newPose = new Pose2d(FIELD_X_DIMENSION_METERS - pose.getX(), pose.getY(),
          Rotation2d.k180deg.minus(pose.getRotation()));
      newStates.add(
          new Trajectory.State(
              state.timeSeconds,
              state.velocityMetersPerSecond,
              state.accelerationMetersPerSecondSq,
              newPose,
              state.curvatureRadPerMeter));
    }
    return new Trajectory(newStates);
  }

  public static Trajectory rotateTrajectory180(Trajectory trajectory) {
    var center = new Translation2d(FIELD_X_DIMENSION_METERS / 2, FIELD_Y_DIMENSION_METERS / 2);
    var states = trajectory.getStates();
    var firstState = states.get(0);
    var firstPose = firstState.poseMeters;
    var newFirstPose = firstPose.rotateAround(center, Rotation2d.k180deg);
    var newStates = new ArrayList<Trajectory.State>();
    newStates.add(
        new Trajectory.State(
            firstState.timeSeconds,
            firstState.velocityMetersPerSecond,
            firstState.accelerationMetersPerSecondSq,
            newFirstPose,
            firstState.curvatureRadPerMeter));

    for (int i = 1; i < states.size(); i++) {
      var state = states.get(i);
      var pose = state.poseMeters;
      var newPose = pose.rotateAround(center, Rotation2d.k180deg);
      newStates.add(
          new Trajectory.State(
              state.timeSeconds,
              state.velocityMetersPerSecond,
              state.accelerationMetersPerSecondSq,
              newPose,
              state.curvatureRadPerMeter));
    }
    return new Trajectory(newStates);
  }
}
