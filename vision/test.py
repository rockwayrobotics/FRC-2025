import cv2
import robotpy_apriltag as at
from wpimath.units import meters

cap = cv2.VideoCapture(0) # video capture source camera (Here webcam of laptop)
ret,frame = cap.read() # return a single frame in variable frame
grey_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

detector = at.AprilTagDetector()
detector.addFamily("tag36h11")
aprilExist = detector.detect(grey_frame)
config = at.AprilTagPoseEstimator.Config(
    tagSize = meters(0.1651),
    fx = 1.0,
    fy = 1.0,
    cx = 320.0,
    cy = 240.0,
)
estimator = at.AprilTagPoseEstimator(config)
print(len(aprilExist))
for tag in aprilExist:
    transform = estimator.estimate(tag)
    print(transform)

field = at.AprilTagFieldLayout("2025-reefscape.json")   
pose = field.getTagPose(aprilExist[0].getId())
print(pose)

cv2.imwrite('images/c1.png',frame)

cap.release()
