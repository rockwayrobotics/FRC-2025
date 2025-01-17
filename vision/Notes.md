# 2025 Notes

## Todo Soon

- [ ] update Rio(s) to 2025 wpilib etc (probably do 2 of 3 for a start)
    - [x] 2024 drive base
    - [ ] benchbot: to be done
- [ ] record readings while driving around a camera on a cart, calc stats etc
- [ ] learn to build our own Apriltag FieldLayout (JSON)
- [ ] learn how to convert AprilTag pose stuff to "cameraToObject" for
      the ComputerVisionUtil.objectToRobotPose() call
      i.e. figure out how to calculate our own robotToCamera Transform3d
- [ ] proof of concept of addVisionMeasurement() with NT Transform3d
- [ ] Python code to send results of AprilTag detection and pose estimation
      over network tables to be shown in Field2d in web dashboard
- [ ] consider doing camera calibration and using it
- [ ] print checkerboard for calibration and document the process
- [ ] more robust camera mounting for R&D (better than cardboard box)
- [ ] acquire USB type A to C cable(s) for powering Rpi from Rio

## Later
- [ ] learn how to extract ambiguous poses to pull out the right one
      Switch from estimate() to estimateOrthogonalIteration()
      to get both pose estimates plus ambiguity (ratio of errors)
      so we can use our own algorithm to resolve the ambiguity.
- [ ] modify drive code to use SimpleMotorFeedforward stuff
- [ ] explore AdvantageKit and possibly adopt their code structure
- [ ] characterize camera parameters/performance
     - frame rate ranges (check if exposure time affects)
     - native resolutions
     - native formats
     - configurable parameters available (e.g. exposure time)


## Done
- [x] set up Samba share on fire1 (user "user", no password)
- [x] try to increase max AprilTag distance at 640x480 again
      Conclusion was we never really changed, and need higher res
      or narrower field of view for real distance.
