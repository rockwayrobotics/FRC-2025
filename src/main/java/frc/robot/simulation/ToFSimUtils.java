package frc.robot.simulation;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;

import java.util.Optional;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.simulation.WorldSimulation.FieldObstacles;

public class ToFSimUtils {
  public static double simulateSensor(Translation2d sensorPos, double sensorAngle, FieldObstacles fieldObstacles) {
    double rayLength = Constants.ToFSensor.MAX_DISTANCE_METERS;
    Translation2d rayEnd = new Translation2d(
        sensorPos.getX() + rayLength * Math.cos(sensorAngle),
        sensorPos.getY() + rayLength * Math.sin(sensorAngle));

    double closestDistance = Double.POSITIVE_INFINITY;
    for (int i = 0; i < fieldObstacles.blueReefVertices.length; i++) {
      Translation2d p1 = fieldObstacles.blueReefVertices[i];
      Translation2d p2 = fieldObstacles.blueReefVertices[(i + 1) % fieldObstacles.blueReefVertices.length];
      var intersection = ToFSimUtils.rayToWallIntersection(sensorPos, rayEnd, p1, p2);
      if (intersection.isPresent()) {
        double distanceMm = Millimeters.convertFrom(sensorPos.getDistance(intersection.get()), Meters);
        closestDistance = Math.min(closestDistance, distanceMm);
      }
    }

    for (int i = 0; i < fieldObstacles.redReefVertices.length; i++) {
      Translation2d p1 = fieldObstacles.redReefVertices[i];
      Translation2d p2 = fieldObstacles.redReefVertices[(i + 1) % fieldObstacles.redReefVertices.length];
      var intersection = ToFSimUtils.rayToWallIntersection(sensorPos, rayEnd, p1, p2);
      if (intersection.isPresent()) {
        double distanceMm = Millimeters.convertFrom(sensorPos.getDistance(intersection.get()), Meters);
        closestDistance = Math.min(closestDistance, distanceMm);
      }
    }

    return closestDistance;
  }

  public static Optional<Translation2d> rayToWallIntersection(
      Translation2d rayStart, Translation2d rayEnd, Translation2d wallStart, Translation2d wallEnd) {
    double x1 = rayStart.getX(), y1 = rayStart.getY();
    double x2 = rayEnd.getX(), y2 = rayEnd.getY();
    double x3 = wallStart.getX(), y3 = wallStart.getY();
    double x4 = wallEnd.getX(), y4 = wallEnd.getY();

    double denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    // Check if lines are parallel
    if (Math.abs(denominator) < 1e-10) {
      return Optional.empty();
    }

    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denominator;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denominator;

    // Check if intersection occurs within both line segments
    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
      double x = x1 + t * (x2 - x1);
      double y = y1 + t * (y2 - y1);
      return Optional.of(new Translation2d(x, y));
    }

    return Optional.empty();
  }
}
