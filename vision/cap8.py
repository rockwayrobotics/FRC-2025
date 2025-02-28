#!/usr/bin/env python3

# TODO: camera calibration https://learnopencv.com/camera-calibration-using-opencv/
# see also https://github.com/tobybreckon/python-examples-cv/blob/master/calibrate_camera.py
# chessboard: https://docs.opencv.org/3.4/pattern.png

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
from cscore import CvSource, VideoMode, CameraServer, MjpegServer

import numpy as np

from keys import NonBlockingConsole

import cv2
import robotpy_apriltag as at

from ntcore import NetworkTableInstance, PubSubOptions
from wpimath.geometry import CoordinateSystem, Transform3d, Translation3d, Rotation3d
from wpimath import units

from libcamera import Transform
import picamera2


degrees = lambda rad: rad * 180 / math.pi

class PiCam:
    def __init__(self, args, id):
        self.cam = picamera2.Picamera2(id)
        self.id = id

        # Default exposure time is 22162 us
        # To get fast fps, need Exposure time lower, buffer_count > 1 (4 is better than 2), and FrameDurationLimits maybe (slight difference)
        vid1 = self.cam.create_video_configuration(
            main=dict(size=SIZE, format='RGB888'),
            lores=dict(size=[320,240], format='RGB888'),
            controls=dict(ExposureTime=11081, FrameRate=args.fps, FrameDurationLimits=(1,17000)), # FrameDurationLimits=(1, 5000)),
            queue=False,
            buffer_count=4,
            transform=Transform(hflip=1, vflip=1),
            )

        cam_config = vid1
        self.cam.align_configuration(cam_config)
        # print('Cam config:\n%s' % '\n'.join(f'{x:>15} = {y}' for x, y in cam_config.items()))
        self.cam.configure(cam_config)

    def capture_array(self, config):
        return self.cam.capture_array(config)

    def stop(self):
        self.cam.stop()
    
    def start(self):
        self.cam.start()

class FakeCam:
    def __init__(self, args, id):
        self.id = id

        image = cv2.imread('camera-fallbacks/cam0.jpg' if self.id == 0 else 'camera-fallbacks/cam1.jpg')
        self.resized = cv2.resize(image, (SIZE[0], SIZE[1]))
        self.resizedLowRes = cv2.resize(image, (320, 240))

    def capture_array(self, config):
        if config == 'main':
            return self.resized
        elif config == 'lores':
            return self.resizedLowRes
        
    def stop(self):
        pass
    
    def start(self):
        pass

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
        **CALS.get(args.cals, CALS.get('640x480')))

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




# imx219 [3280x2464] (/base/soc/i2c0mux/i2c@1/imx219@10)
# 'SRGGB10_CSI2P' :  640x480  [206.65 fps - (1000, 752)/1280x960 crop]
#                   1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
#                   1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
#                   3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
#        'SRGGB8' :  640x480  [206.65 fps - (1000, 752)/1280x960 crop]
#                   1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
#                   1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
#                   3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]


CALS = {
    # images1
    # fx = 717.428307,
    # fy = 712.943769,
    # cx = 365.509738,
    # cy = 219.080481,

    # images2 640x480
    '640x480': dict(
        fx = 813.002665,
        fy = 814.367913,
        cx = 340.340811,
        cy = 248.727651,
    ),

    # images4 1456x1088
    '1456x1088': dict(
        fx = 1757.669488,
        fy = 1762.782233,
        cx = 736.690867,
        cy = 557.428635,
    ),
}

def cam2Field(fieldPos, camPos):
    tagInCameraFrame = Transform3d(camPos.x, camPos.y, camPos.z, Rotation3d(-camPos.rotation().x - math.pi, -camPos.rotation().y, camPos.rotation().z - math.pi))
    # Convert from East-Down-North to North-West-Up
    tagInCameraFrameNWU = CoordinateSystem.convert(tagInCameraFrame, CoordinateSystem.EDN(), CoordinateSystem.NWU())
    # Add the field-relative tag position to the inverse of the camera-to-tag transform and return it
    result = (fieldPos + tagInCameraFrameNWU.inverse()).toPose2d()
    return result

def main():
    try:
        cam0 = PiCam(args, 0)
    except:
        cam0 = FakeCam(args, 0)
    try:
        cam1 = PiCam(args, 1)
    except:
        cam1 = FakeCam(args, 1)

    cam = cam0

    nt = NetworkTableInstance.getDefault()
    nt.setServerTeam(8089)
    nt.startClient4("pi cam")

    source = CvSource("cam", VideoMode.PixelFormat.kBGR, SIZE[0], SIZE[1], int(args.fps))
    mjpegServer = MjpegServer("mjpeg", 8081)
    mjpegServer.setSource(source)
    mjpegTopic = nt.getStringArrayTopic("/CameraPublisher/PiCam/streams").publish(PubSubOptions())

    selectCameraTopic = nt.getStringTopic("/CameraPublisher/PiCam/selected")
    selectCameraPublisher = selectCameraTopic.publish(PubSubOptions())
    selectCameraPublisher.set("fore")
    selectCameraSubscriber = selectCameraTopic.subscribe("fore")

    camAutoDeadbandTopic = nt.getDoubleTopic("/Tuning/Camera Auto Deadband")
    camAutoDeadbandSubscriber = camAutoDeadbandTopic.subscribe(0)    

    rightVelocitySubscriber = nt.getDoubleTopic("/AdvantageKit/Drive/RightVelocityMetersPerSec").subscribe(0)
    leftVelocitySubscriber = nt.getDoubleTopic("/AdvantageKit/Drive/LeftVelocityMetersPerSec").subscribe(0)

    mjpegTopic.set(["mjpg:http://10.80.89.11:8081/?action=stream"])

    cam0.start()
    cam1.start()

    aprilDetector = AprilTagDetection(args, nt)

    # mjpegServer = CameraServer.startAutomaticCapture(source)
    # print(mjpegServer.getPort())

    # time.sleep(0.1)

    console = NonBlockingConsole()

    if args.save:
        cam0stream = open('cam0stream.mjpeg', 'ab')
        cam1stream = open('cam1stream.mjpeg', 'ab')

    try:
        now = start = time.time()
        reported = start
        count = 0
        fps = 0
        height = SIZE[1] * 2 // 3
        print('SIZE', SIZE, 'height', height, end='\r\n')

        while now - start < args.time:
            currentCam = selectCameraSubscriber.get()
            if currentCam == 'fore':
                cam = cam0
            elif currentCam == 'aft':
                cam = cam1
            elif currentCam == 'auto':
                leftVelocity = leftVelocitySubscriber.get()
                rightVelocity = rightVelocitySubscriber.get()
                deadbandThreshold = camAutoDeadbandSubscriber.get()
                #print(f'deadbandThreshold={deadbandThreshold}', end = '\r\n')

                if abs(leftVelocity) > deadbandThreshold and abs(rightVelocity) > deadbandThreshold:
                    if leftVelocity > 0 and rightVelocity > 0:
                        cam = cam0
                    else:
                        cam = cam1

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
                    cam = cam1 if cam == cam0 else cam0
                    print(f'Switched to camera {cam.id}')

    finally:
        del console
        print()
        cam0.stop()
        cam1.stop()
        if args.save:
            cam0stream.close()
            cam1stream.close()

if __name__ == '__main__':
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

    args = parser.parse_args()
    if args.cals is None:
        args.cals = args.res
    SIZE = tuple(int(x) for x in args.res.split('x'))

    main()
