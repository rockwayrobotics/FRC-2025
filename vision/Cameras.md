# Cameras

## Calibration

Cameras generally have various types of distortion, as well as
camera-specific parameters such as the focal length, native
resolutions, etc.

To get accurate pose estimates from the AprilTag library we need
to supply four parameters, with x/y values for each of the focal
length and the camera centre.

These values are specified in pixels, which means we need to use
the same resolution during calibration as we'll use in operation,
or at least we'll need to know how to calibrate with one resolution
and then convert the calibration results for a different resolution.
(We don't know yet what that would look like.)

## Resources

- [Checkerboard Collection](https://markhedleyjones.com/projects/calibration-checkerboard-collection)
  contains some pregenerated checkerboards as well as a tool
  to generate SVG images for any type of checkerboard.

- [Our 6x9 checkerboard](docs/Checkerboard-A4-9x6.pdf) contains
  our "standard" checkerboard, poorly named (someone please reverse
  the 9x6 to 6x9), and intended to be used with the cal_cam.py script.

## How to Calibrate


