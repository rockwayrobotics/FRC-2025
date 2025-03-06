import cv2
import math

from wpimath.geometry import CoordinateSystem, Transform3d, Translation3d, Rotation3d, Pose2d, Pose3d
import robotpy_apriltag as at

fieldHeight = 8.05
fieldWidth = 17.55
widthScale = 1
heightScale = 1
fieldLayout = at.AprilTagFieldLayout("2025-reefscape.json")

# Convert from field coordinates in m to image coordinates in pixels
def f2i(xy):
    return (round(widthScale * xy.x), round(heightScale * (fieldHeight - xy.y)))

def myTransform(tagCamera, tagField):
    # FIXME: This is the rotation that I do NOT understand. I think it should somehow be inverting the rotation so that we get the camera orientation rather than the tag orientation.
    tagInCameraFrame = Transform3d(tagCamera.x, tagCamera.y, tagCamera.z, Rotation3d(-tagCamera.rotation().x - math.pi, -tagCamera.rotation().y, tagCamera.rotation().z - math.pi))
    # Convert from East-Down-North to North-West-Up
    tagInCameraFrameNWU = CoordinateSystem.convert(tagInCameraFrame, CoordinateSystem.EDN(), CoordinateSystem.NWU())
    # Add the field-relative tag position to the inverse of the camera-to-tag transform and return it
    return tagField + tagInCameraFrameNWU.inverse()

def drawTag(image, pose):
    # Draw a line that approximates the tag (this is 40cm wide rather than 6.5in but easier to view)
    lineStart = pose + Transform3d(0, -0.2, 0, Rotation3d())
    lineEnd = pose + Transform3d(0, 0.2, 0, Rotation3d())
    cv2.line(image, f2i(lineStart), f2i(lineEnd), (0, 0, 255), 5)

    # Draw a line straight forward from the tag
    lineStart = pose + Transform3d(0.3, 0, 0, Rotation3d())
    lineEnd = pose
    cv2.line(image, f2i(lineStart), f2i(lineEnd), (0, 0, 255), 5)

    # Put a circle in the center of the tag
    cv2.circle(image, f2i(pose), 8, (0, 255, 0), -1)

def drawRobot(image, pose):
    # Draw a square of side length 0.3m
    size = 0.3 / 2
    lineStart = pose + Transform3d(-size, -size, 0, Rotation3d())
    lineEnd = pose + Transform3d(-size, size, 0, Rotation3d())
    cv2.line(image, f2i(lineStart), f2i(lineEnd), (255, 128, 0), 5)
    lineStart = pose + Transform3d(-size, size, 0, Rotation3d())
    lineEnd = pose + Transform3d(size, size, 0, Rotation3d())
    cv2.line(image, f2i(lineStart), f2i(lineEnd), (255, 128, 0), 5)

    # This is the front, use a different colour
    lineStart = pose + Transform3d(size, size, 0, Rotation3d())
    lineEnd = pose + Transform3d(size, -size, 0, Rotation3d())
    cv2.line(image, f2i(lineStart), f2i(lineEnd), (255, 255, 0), 5)
    lineStart = pose + Transform3d(size, -size, 0, Rotation3d())
    lineEnd = pose + Transform3d(-size, -size, 0, Rotation3d())
    cv2.line(image, f2i(lineStart), f2i(lineEnd), (255, 128, 0), 5)

    # Put a circle in the center
    cv2.circle(image, f2i(pose), 8, (255, 128, 0), -1)

def main():
    global widthScale, heightScale
    image = cv2.imread('tagfield.png')
    # Read the width and height of the image so we can determine our scale
    imageHeight, imageWidth = image.shape[:2]
    widthScale = imageWidth / fieldWidth
    heightScale = imageHeight / fieldHeight

    # Draw all the tags on the field with their poses
    for i in range(1, 23):
        tagField = fieldLayout.getTagPose(i)
        drawTag(image, tagField)

    # Calculate some sample poses that are 2m away from the tag but rotated
    for i in range(-60, 60, 30):
        tagCamera = Transform3d(0, 0, 2, Rotation3d(0, i * math.pi / 180, 0))
        robotPose = myTransform(tagCamera, fieldLayout.getTagPose(2))
        drawRobot(image, robotPose)

    # Calculate some sample poses that are 2m away from a different tag and rotated
    for i in range(-60, 61, 30):
        tagCamera = Transform3d(0, 0, 2, Rotation3d(0, i * math.pi / 180, 0))
        robotPose = myTransform(tagCamera, fieldLayout.getTagPose(7))
        drawRobot(image, robotPose)
   
    # Show the window and wait for a keypress
    cv2.imshow('Field', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
   main() 
