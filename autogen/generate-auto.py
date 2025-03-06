import json
import math

# As best I can tell, the starting line is at coordinate x = 7.58 m
starting_x = 7.58

def waypointToPathPlanner(w):
  return {
    'anchor': {
      'x': w['x'],
      'y': w['y'],
    },
    
  }

def write_json(start, end, angle_deg):
  output = {
    'version': '2025.0',
    'waypoints': [],
    'rotationTargets': [],
    'constraintZones': [],
    'pointTowardsZones': [],
    'eventMarkers': [],
    'globalConstraints': {
      'maxVelocity': 0.5,
      'maxAcceleration': 3.0,
      'maxAngularVelocity': 540.0,
      'maxAngularAcceleration': 720.0,
      'nominalVoltage': 12.0,
      'unlimited': False,
    },
    'goalEndState': {
      'velocity': 0,
      'rotation': 0.0,
    },
    'reversed': False,
    'idealStartingState': {
      'velocity': 0,
      'rotation': 0.0,
    },
    'useDefaultConstraints': True,
  }

  output['waypoints'] += [{
    'anchor': {
      'x': start[0],
      'y': start[1],
    },
    'prevControl': None,
    'nextControl': {
      'x': start[0] + 0.25 * math.cos(angle_deg * math.pi / 180.0),
      'y': start[1] + 0.25 * math.sin(angle_deg * math.pi / 180.0),
    },
    'isLocked': False,
    'linkedName': None,
  }]
  output['waypoints'] += [{
    'anchor': {
      'x': end[0],
      'y': end[1],
    },
    'prevControl': {
      'x': end[0] - 0.25 * math.cos(angle_deg * math.pi / 180.0),
      'y': end[1] - 0.25 * math.sin(angle_deg * math.pi / 180.0),
    },
    'nextControl': None,
    'isLocked': False,
    'linkedName': None,
  }]
  json_data = json.dumps(output)
  print(json_data)

def main(args):
  with open('../vision/2025-reefscape.json', 'r') as file:
    data = json.load(file)

  tag_pose = [element['pose'] for element in data['tags'] if element['ID'] == args.t]
  tag_pos = (tag_pose[0]['translation']['x'], tag_pose[0]['translation']['y'])

  # We have the quaternion and we should really use a library to turn it into a z-rotation
  q = tag_pose[0]['rotation']['quaternion']
  # However... if x and y are basically 0, θ = 2 * atan2(z, w)
  z_rot = 2 * math.atan2(q['Z'], q['W'])
  normal = (math.cos(z_rot), math.sin(z_rot))
  if (abs(normal[0]) < 1e-6):
    print('Need to add intermediate point')

  end = (tag_pos[0] + normal[0] * 0.5, tag_pos[1] + normal[1] * 0.5)
  slope = -normal[0] / normal[1]
  angle = math.atan2(-normal[0], normal[1])
  angle_deg = angle * 180.0 / math.pi
  start = (args.x, end[1] - slope * (end[0] - args.x))

  print(start, end, angle_deg)
  print(f'Trajectory traj = new TrajectoryGenerator.generateTrajectory(new Pose2d({start[0]:.3f}, {start[1]:.3f}, Rotation2d.fromDegrees({angle_deg:.2f})), List.of(), new Pose2d({end[0]:.3f}, {end[1]:.3f}, Rotation2d.fromDegrees({angle_deg:.2f})), config);')
  write_json(start, end, angle_deg)

  # Trajectory traj = new TrajectoryGenerator.generateTrajectory(new Pose2d(x, y, Rotation2d.fromDegrees(degrees)), List.of(new Translation2d(x, y), new Translation2d(x, y)), new Pose2d(x, y, Rotation2d.fromDegrees(degrees)), config);
    

if __name__ == '__main__':
  import argparse
  parser = argparse.ArgumentParser()
  parser.add_argument('-x', type=float, default=starting_x)
  parser.add_argument('--algae', action='store_true')

  # AprilTags on blue side are 17-22.
  parser.add_argument('-t', type=int, default=17)

  args = parser.parse_args()
  main(args)
