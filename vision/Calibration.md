## Calibration

To calibrate the a camera, we need to have a checkerboard pattern printed and mounted it to a fixed board. With the current sequence, you need at least an 8x5 internal area (don't count the border).
The calibration process is specific to a given camera, lens and resolution, so if any of these change, you will need to re-run calibration.

To calibrate:
1. Generate test images. You may want to delete or move the current images/ directory away, since this process will write to it. To get images, you can do this with `python cap4.py --res WxH`. Open a browser to http://fire1.local:8000/index.html and make sure you can see the camera images. Then make sure your terminal is selected, and it will save pictures to the images/ folder whenever you press the spacebar. You want about 10 images with the chessboard visible, and in our experience the chessboard should take up at least 1/9 of the viewport. Smaller chessboards do not work well for calibration.
2. Get calibration parameters from the images. Run `python cal_cam.py --dir images --res WxH`. This will generate some output, but you want the last four lines, that will look something like:
    fx = 895.628914,
    fy = 899.300743,
    cx = 655.965312,
    cy = 344.614969,
For reference, the cx,cy values should be approximately half of the width and height, and the fx/fy parameters should be roughly equal (assuming spherical lens?)
These are the calibration parameters that you need for the AprilTagPoseEstimator.Config constructor.
