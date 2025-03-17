import json
import math
import sys

# As best I can tell, the starting line is at coordinate x = 7.58 m
starting_x = 7.58

# From the cad, the distance between the bar centers is 328.6 mm
bar_distance = 0.3286

# From the cad, the wall length is 940.9 mm
wall_length = 0.9409

# length of control points in PathPlanner, does not affect Java
control_length = 0.25

# From the cad, the chute shooting center is 64.151 mm in front of the robot center
chute_distance = 0.064151


class Vector:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, b: "Vector"):
        return Vector(self.x + b.x, self.y + b.y)

    def __sub__(self, b: "Vector"):
        return Vector(self.x - b.x, self.y - b.y)

    def __mul__(self, b):
        return Vector(b * self.x, b * self.y)

    def __rmul__(self, b):
        return Vector(b * self.x, b * self.y)

    def __truediv__(self, b):
        if b != 0:
            return Vector(self.x / b, self.y / b)
        else:
            return Vector(self.x / 0.001, self.y / 0.001)

    def __str__(self):
        return f"({self.x}, {self.y})"

    def dot(self, b: "Vector"):
        return self.x * b.x + self.y * b.y

    def length(self):
        return math.sqrt(self.x**2 + self.y**2)

    def unit(self):
        return self / self.length()

    def as_tuple(self):
        coord = (self.x, self.y)
        return coord


def write_json(start: Vector, end: Vector, intermediate=[]):
    output = {
        "version": "2025.0",
        "waypoints": [],
        "rotationTargets": [],
        "constraintZones": [],
        "pointTowardsZones": [],
        "eventMarkers": [],
        "globalConstraints": {
            "maxVelocity": 0.5,
            "maxAcceleration": 3.0,
            "maxAngularVelocity": 540.0,
            "maxAngularAcceleration": 720.0,
            "nominalVoltage": 12.0,
            "unlimited": False,
        },
        "goalEndState": {
            "velocity": 0,
            "rotation": 0.0,
        },
        "reversed": False,
        "idealStartingState": {
            "velocity": 0,
            "rotation": 0.0,
        },
        "useDefaultConstraints": True,
    }

    points = [None, start] + [p for p in intermediate] + [end, None]
    for prev, current, next in zip(points, points[1:], points[2:]):
        if prev is None:
            dist = math.sqrt((next.x - current.x) ** 2 + (next.y - current.y) ** 2)
            prevControl = None
            nextControl = {
                "x": current.x + control_length * (next.x - current.x) / dist,
                "y": current.y + control_length * (next.y - current.y) / dist,
            }
        elif next is None:
            dist = math.sqrt((current.x - prev.x) ** 2 + (current.y - prev.y) ** 2)
            prevControl = {
                "x": current.x + control_length * (prev.x - current.x) / dist,
                "y": current.y + control_length * (prev.y - current.y) / dist,
            }
            nextControl = None
        else:
            dist = math.sqrt((next.x - prev.x) ** 2 + (next.y - prev.y) ** 2)
            prevControl = {
                "x": current.x + control_length * (prev.x - next.x) / dist,
                "y": current.y + control_length * (prev.y - next.y) / dist,
            }
            nextControl = {
                "x": current.x + control_length * (next.x - prev.x) / dist,
                "y": current.y + control_length * (next.y - prev.y) / dist,
            }

        output["waypoints"] += [
            {
                "anchor": {
                    "x": current.x,
                    "y": current.y,
                },
                "prevControl": prevControl,
                "nextControl": nextControl,
                "isLocked": False,
                "linkedName": None,
            }
        ]

    json_data = json.dumps(output)
    print(json_data)


def write_java(start, end, start_angle_deg, end_angle_deg, intermediate=[]):
    """
    Generate something like:
      Trajectory traj = new TrajectoryGenerator.generateTrajectory(
        new Pose2d(x, y, Rotation2d.fromDegrees(degrees)),
        List.of(new Translation2d(x, y), new Translation2d(x, y)),
        new Pose2d(x, y, Rotation2d.fromDegrees(degrees)),
        config
      );

    """
    startPose = f"new Pose2d({start.x:.3f}, {start.y:.3f}, Rotation2d.fromDegrees({start_angle_deg:.2f}))"
    endPose = f"new Pose2d({end.x:.3f}, {end.y:.3f}, Rotation2d.fromDegrees({end_angle_deg:.2f}))"
    intermediatePoints = ", ".join(
        f"new Translation2d({p.x:.3f}, {p.y:.3f})" for p in intermediate
    )
    print(
        f"""
    Trajectory traj = new TrajectoryGenerator.generateTrajectory(
      {startPose},
      List.of({intermediatePoints}),
      {endPose},
      config
    );"""
    )


def main(args):
    with open("../vision/2025-reefscape.json", "r") as file:
        data = json.load(file)

    tag_pose = [element["pose"] for element in data["tags"] if element["ID"] == args.t]
    tag_pos = (tag_pose[0]["translation"]["x"], tag_pose[0]["translation"]["y"])

    # We have the quaternion and we should really use a library to turn it into a z-rotation
    q = tag_pose[0]["rotation"]["quaternion"]
    # However... if x and y are basically 0, θ = 2 * atan2(z, w)
    z_rot = 2 * math.atan2(q["Z"], q["W"])
    z_rot_deg = math.degrees(z_rot)

    # this is a unit vector pointing out from the tag
    normal = Vector(math.cos(z_rot), math.sin(z_rot))
    # this is directly in front of the tag
    distance_from_wall = 0.41 + (args.d / 1000.0)
    in_front_of_tag = Vector(tag_pos[0] + normal.x * distance_from_wall, tag_pos[1] + normal.y * distance_from_wall)

    waypoints = []

    if args.algae:
        end = in_front_of_tag
        if z_rot_deg > 90 and z_rot_deg < 270:
            print(
                "Not supported - wall is facing away from start position, need multiple intermediate points for algae"
            )
            print(f"Your target point is probably: {end}")
            sys.exit(1)
        else:
            start = Vector(args.x, end.y - math.tan(math.pi + z_rot) * (end.x - args.x))
            angle_deg = 180 + z_rot_deg
            print(angle_deg)
            if start.y < 0.5:
                print("Start point off the field bottom, need intermediate point")
                print(
                    f"Your target point is probably: {end}, and you want to approach from {start}"
                )
                sys.exit(1)
            elif start.y > 7.5:
                print("Start point off the field top, need intermediate point")
                print(
                    f"Your target point is probably: {end}, and you want to approach from {start}"
                )
                sys.exit(1)

        angle_deg = 180 + z_rot_deg
        print(start, end, angle_deg)
        write_json(start, end, waypoints)
        write_java(start, end, angle_deg, angle_deg, waypoints)
    else:
        if abs(normal.y) > 1e-6:
            # this is the angle of the wall
            angle = z_rot - math.pi / 2 if z_rot > math.pi else z_rot + math.pi / 2
            angle_deg = angle * 180.0 / math.pi
            print(f"Approaching wall with angle {angle_deg:.2f} degrees")
            if args.far:
              distance_along_wall = chute_distance - bar_distance / 2
            elif args.trough:
              distance_along_wall = chute_distance + bar_distance
            else:
              distance_along_wall = chute_distance + bar_distance / 2
            end = in_front_of_tag - distance_along_wall * Vector(
                math.cos(angle), math.sin(angle)
            )

            start = Vector(args.x, end.y - math.tan(angle) * (end.x - args.x))
        else:
            # wall slope is vertical so figure out where we want to use as our intermediate point
            angle = z_rot - math.pi / 2 if z_rot > math.pi else z_rot + math.pi / 2
            angle_deg = angle * 180.0 / math.pi
            print(f"Approaching wall with angle {angle_deg:.2f} degrees")
            if args.far:
              distance_along_wall = chute_distance - bar_distance / 2
            elif args.trough:
              distance_along_wall = chute_distance + bar_distance
            else:
              distance_along_wall = chute_distance + bar_distance / 2
            end = in_front_of_tag - distance_along_wall * Vector(
                math.cos(angle), math.sin(angle)
            )

            print(
                "Not supported - wall is perpendicular to starting line, need intermediate point"
            )
            print(f"Your target point is probably: {end}")
            sys.exit(1)

        print(start, end, angle_deg)
        write_json(start, end, waypoints)
        write_java(start, end, angle_deg, angle_deg, waypoints)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    # Starting x could be changed, this is robot center
    parser.add_argument("-x", type=float, default=starting_x)
    # Assumes coral if not set
    parser.add_argument("--algae", action="store_true")
    # Assumes near if not set
    parser.add_argument("--far", action="store_true")

    # AprilTags on blue side are 17-22.
    parser.add_argument("-t", type=int, default=17)
    # Intended distance from wall in millimeters
    parser.add_argument("-d", type=float, default=30)
    # Shoot early for trough
    parser.add_argument("--trough", action="store_true")

    args = parser.parse_args()
    main(args)
