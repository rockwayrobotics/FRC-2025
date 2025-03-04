#!/usr/bin/env python3

# pylint: disable=unused-import,invalid-name,trailing-whitespace,missing-module-docstring
# pylint: disable=missing-function-docstring,missing-class-docstring,no-member
# pylint: disable=consider-using-with,too-few-public-methods

import datetime as dt
import io
import logging
import socketserver
from http import server
import threading
import math
import os
import platform
import time
import traceback
from cscore import CvSource, VideoMode, CameraServer, MjpegServer

import numpy as np

import cv2
import robotpy_apriltag as at

from ntcore import NetworkTableInstance, PubSubOptions
from wpimath.geometry import CoordinateSystem, Transform3d, Translation3d, Rotation3d
from wpimath import units

try:
    from libcamera import Transform
    import picamera2
    PI = True
except ImportError:
    PI = False

from . import cam
from .keys import NonBlockingConsole


class AprilTagDetection:
    def __init__(self, args, nt):
        self.det = at.AprilTagDetector()
        self.det.addFamily('tag36h11') # default: bitsCorrected=2
        cfg = self.det.getConfig()
        cfg.quadDecimate = args.dec
        cfg.numThreads = args.threads
        cfg.decodeSharpening = 0.25 # margin jumps a lot with 1.0
        # cfg.quadSigma = 0.8
        self.det.setConfig(cfg)
        # print(f'Apriltags:\n\t{cfg.decodeSharpening=} {cfg.numThreads=} {cfg.quadDecimate=}\n'
        #     f'\t{cfg.quadSigma=} {cfg.refineEdges=} {cfg.debug=}')
        # q = det.getQuadThresholdParameters()
        # print(f'Quad Threshold:\n\t{degrees(q.criticalAngle)=} {q.deglitch=} {q.maxLineFitMSE=}\n'
        #     f'\t{q.maxNumMaxima=} {q.minClusterPixels=} {q.minWhiteBlackDiff=}')

        self.field = at.AprilTagFieldLayout("2025-reefscape.json")
        config = at.AprilTagPoseEstimator.Config(
            tagSize = units.inchesToMeters(6.5), # 16.51cm
            **CALS.get(args.cals, CALS.get('640x480')).get_dict()
        )

        self.estimator = at.AprilTagPoseEstimator(config)

        self.tagPose = nt.getFloatArrayTopic(
            "/Shuffleboard/Drivebase/Field2d/Robot"
        ).publish(PubSubOptions())  # Uses default options
        self.tagPose2 = nt.getFloatArrayTopic(
            "/Shuffleboard/Drivebase/Field2d/Robot2"
        ).publish(PubSubOptions())  # Uses default options
        self.ambiguityTopic = nt.getFloatTopic(
            "/Shuffleboard/Drivebase/Field2d/Ambiguity"
        ).publish(PubSubOptions())  # Uses default options

    def detect(self, arr):
        # img = arr[:height,:]
        img = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
        tags = self.det.detect(img)

        if bool(tags):
            tag = sorted(tags, key=lambda x: -x.getDecisionMargin())[0]
            tid = tag.getId()
            absolutePose = self.field.getTagPose(tid)
            # tf = estimator.estimate(tag)
            poseEstimate = self.estimator.estimateOrthogonalIteration(tag, 50)
            ambiguity = poseEstimate.getAmbiguity()
            pose1 = cam2Field(absolutePose, poseEstimate.pose1)
            pose2 = cam2Field(absolutePose, poseEstimate.pose2)
            print(f'\rAmbiguity={ambiguity}')
            # _buf = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            # print(tag.getCorners(_buf))
            self.ambiguityTopic.set(ambiguity * 100)
            if pose1 is not None:
                print(f'\r{poseEstimate.error1} Pose1={round(pose1.x,2)},{round(pose1.y,2)},{round(pose1.rotation().degrees())}')
                self.tagPose.set([pose1.x, pose1.y, pose1.rotation().radians()])
            if pose2 is not None and ambiguity > 0:
                print(f'\r{poseEstimate.error2} Pose2={round(pose2.x,2)},{round(pose2.y,2)},{round(pose2.rotation().degrees())}')
                self.tagPose2.set([pose2.x, pose2.y, pose2.rotation().radians()])
            if pose1 is not None and pose2 is not None:
                print(f'\rDistance: {pose1.translation().distance(pose2.translation())}')
            return True
        return False


class CamCal:
    def __init__(self, fx, fy, cx, cy):
        self.array = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], np.float64)
        self.value = { "fx":fx, "fy":fy, "cx":cx, "cy": cy }
    def get_array(self):
        return self.array
    def get_dict(self):
        return self.value

CALS = {
    # images2 640x480
    '640x480': CamCal(fx=813.002665, fy=814.367913, cx=340.340811, cy=248.727651),

    # images4 1456x1088
    '1456x1088': CamCal(fx=1757.669488, fy=1762.782233, cx=736.690867, cy=557.428635),
}

def cam2Field(fieldPos, camPos):
    tagInCameraFrame = Transform3d(camPos.x, camPos.y, camPos.z,
       Rotation3d(-camPos.rotation().x - math.pi,
                  -camPos.rotation().y, camPos.rotation().z - math.pi))
    # Convert from East-Down-North to North-West-Up
    tagInCameraFrameNWU = CoordinateSystem.convert(
        tagInCameraFrame, CoordinateSystem.EDN(), CoordinateSystem.NWU())

    # Add field-relative tag position to the inverse of the camera-to-tag transform and return it.
    result = (fieldPos + tagInCameraFrameNWU.inverse()).toPose2d()
    return result


def main():
    args = get_args()
    res = args.res

    cam0 = cam.Camera(0, args.res, args.fps)
    cam1 = cam.Camera(1, args.res, args.fps)

    driver_cam = cam0

    nt = NetworkTableInstance.getDefault()
    nt.setServerTeam(8089)
    nt.startClient4("pi cam")

    source = CvSource("cam", VideoMode.PixelFormat.kBGR, res[0], res[1], int(args.fps))
    mjpegServer = MjpegServer("mjpeg", 8081)
    mjpegServer.setSource(source)
    mjpegTopic = nt.getStringArrayTopic("/CameraPublisher/PiCam/streams").publish(PubSubOptions())

    selectCameraTopic = nt.getStringTopic("/CameraPublisher/PiCam/selected")
    selectCameraPub = selectCameraTopic.publish(PubSubOptions())
    selectCameraPub.set("fore")
    selectCameraSub = selectCameraTopic.subscribe("fore")

    camAutoDeadbandTopic = nt.getDoubleTopic("/Tuning/Camera Auto Deadband")
    camAutoDeadbandSub = camAutoDeadbandTopic.subscribe(0)    

    rightVelocitySub = nt.getDoubleTopic("/AdvantageKit/Drive/RightVelocityMetersPerSec").subscribe(0)
    leftVelocitySub = nt.getDoubleTopic("/AdvantageKit/Drive/LeftVelocityMetersPerSec").subscribe(0)

    mjpegTopic.set(["mjpg:http://10.80.89.11:8081/?action=stream"])

    cam0.start()
    cam1.start()

    aprilDetector = AprilTagDetection(args, nt)

    # mjpegServer = CameraServer.startAutomaticCapture(source)
    # print(mjpegServer.getPort())

    console = NonBlockingConsole()

    if args.save:
        cam0stream = open('cam0stream.mjpeg', 'ab')
        cam1stream = open('cam1stream.mjpeg', 'ab')

    try:
        now = start = time.time()
        reported = start
        count = 0
        fps = 0
        height = res[1] * 2 // 3
        print('res', res, 'height', height, end='\r\n')

        while now - start < args.time:
            currentCam = selectCameraSub.get()
            if currentCam == 'fore':
                driver_cam = cam0
            elif currentCam == 'aft':
                driver_cam = cam1
            elif currentCam == 'auto':
                leftVelocity = leftVelocitySub.get()
                rightVelocity = rightVelocitySub.get()
                deadbandThreshold = camAutoDeadbandSub.get()
                #print(f'deadbandThreshold={deadbandThreshold}', end = '\r\n')

                if abs(leftVelocity) > deadbandThreshold and abs(rightVelocity) > deadbandThreshold:
                    if leftVelocity > 0 and rightVelocity > 0:
                        driver_cam = cam0
                    elif leftVelocity < 0 and rightVelocity < 0:
                        driver_cam = cam1

            count += 1
            now = time.time()

            arr0 = cam0.capture_array('main')
            arr1 = cam1.capture_array('main')
            #dashboard_arr = cam.capture_array('lores')
            if args.save:
                result0, encode0 = cv2.imencode('.jpg', arr0)
                result1, encode1 = cv2.imencode('.jpg', arr1)

                if result0:
                    cam0stream.write(encode0)
                if result1:
                    cam1stream.write(encode1)

            dashboard_arr = cv2.resize(arr0 if cam == cam0 else arr1, (320, 240))
            
            source.putFrame(dashboard_arr)

            aprilDetector.detect(arr0)
            aprilDetector.detect(arr1)

            key = console.get_key()

            if now - reported > 1:
                fps = count / (now - reported)
                reported = now
                count = 0
                print(f'FPS: {fps}', end='\r\n')

            if key is not None:
                if key.lower() == 'q':
                    break
                elif key.lower() == 'c':
                    if driver_cam == cam0:
                        driver_cam = cam1
                        selectCameraPub.set("aft")
                    else:
                        driver_cam = cam0
                        selectCameraPub.set("fore")
                    print(f'Switched to camera {driver_cam.num}')

    finally:
        del console # this should restore original kb settings
        print('exiting')
        cam0.stop()
        cam1.stop()
        if args.save:
            print('closing files')
            cam0stream.close()
            cam1stream.close()

        # # These are an attempt to speed up shutdown, which currently
        # # stalls for some reason after we return from this routine.
        # mjpegTopic.close()
        # nt.stopServer()
        # source.setConnected(False)
        # nt.stopClient()

        # import threading
        # print(threading.enumerate())
        # print(dir(nt))

        print('exiting main')


def get_args():
    # pylint: disable=import-outside-toplevel
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--res', default='640x480')
    parser.add_argument('--dec', type=int, default=2)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--fps', type=float, default=60.0)
    parser.add_argument('--time', type=float, default=10000.0)
    parser.add_argument('--cals')
    parser.add_argument('--save', action='store_true')

    # This is a bad way to do this, shoving these into the global
    # namespace, but it's a temporary hack to avoid having to change some
    # of the original code which did that as a shortcut.
    args = parser.parse_args()
    if args.cals is None:
        args.cals = args.res
    args.res = tuple(int(x) for x in args.res.split('x'))
    return args


if __name__ == '__main__':
    main()
