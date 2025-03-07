#!/usr/bin/env python3

# pylint: disable=invalid-name,trailing-whitespace,missing-module-docstring
# pylint: disable=missing-function-docstring,missing-class-docstring,no-member
# pylint: disable=consider-using-with,too-few-public-methods

import datetime as dt
import io
import logging
import threading
import math
import os
import time
import traceback
from cscore import CvSource, VideoMode, CameraServer, MjpegServer

import cv2

from ntcore import NetworkTableInstance, PubSubOptions
from wpimath.geometry import CoordinateSystem, Transform3d, Translation3d, Rotation3d, Pose3d
from wpimath import units

try:
    from libcamera import Transform
    import picamera2
    PI = True
except ImportError:
    PI = False

from . import cam
from .apriltag import AprilTagDetection
from .keys import NonBlockingConsole
from .recording import VideoEncoder


def main():
    args = get_args()

    cam0 = cam.get_camera(0, args.res, fps=args.fps, flip=args.cam0flip)
    cam1 = cam.get_camera(1, args.res, fps=args.fps, flip=args.cam1flip)

    driver_cam = cam0

    nt = NetworkTableInstance.getDefault()
    if args.serve:
        print('starting test NT server')
        nt.startServer()
    else:
        nt.setServerTeam(8089)
        nt.startClient4("pi cam")

    source = CvSource("cam", VideoMode.PixelFormat.kBGR, args.res[0], args.res[1], int(args.fps))
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

    # pick calibration, falling back on built-in cal for current resolution
    # TODO: support reading from a JSON file with lookup by camera id
    # if we can figure out a way of identifying individual cameras, and
    # possibly account for flip=True separately in case that affects the
    # values (or can we just "rotate" the centre/focal point values?)
    try:
        cal = cam.CALS[args.cals]
    except KeyError:
        cal = cam.CALS[args.res]

    aprilDetector = AprilTagDetection(args, nt, cal=cal)

    # mjpegServer = CameraServer.startAutomaticCapture(source)

    if os.environ.get('INVOCATION_ID') is not None:
        class console:
            @staticmethod
            def get_key(): pass
    else:
        console = NonBlockingConsole()

    streams = []
    if args.save:
        for i in [0, 1]:
            stream = VideoEncoder(i, fps=args.fps,
                width=args.res[0], height=args.res[1], quality=args.quality,
                debug=args.debug)
            streams.append(stream)

    try:
        now = start = time.time()
        reported = start
        count = 0
        fps = 0
        print('res', args.res)

        while True:
            currentCam = selectCameraSub.get()
            if currentCam == 'fore':
                if driver_cam is not cam0:
                    print('selecting cam0 (fore)')
                driver_cam = cam0
            elif currentCam == 'aft':
                if driver_cam is not cam1:
                    print('selecting cam1 (aft)')
                driver_cam = cam1
            elif currentCam == 'auto':
                leftVelocity = leftVelocitySub.get()
                rightVelocity = rightVelocitySub.get()
                deadbandThreshold = camAutoDeadbandSub.get()
                #print(f'deadbandThreshold={deadbandThreshold}', end = '\r\n')

                if abs(leftVelocity) > deadbandThreshold and abs(rightVelocity) > deadbandThreshold:
                    if leftVelocity > 0 and rightVelocity > 0:
                        if driver_cam is not cam0:
                            print('selecting cam0 (fore)')
                        driver_cam = cam0
                    elif leftVelocity < 0 and rightVelocity < 0:
                        if driver_cam is not cam1:
                            print('selecting cam1 (aft)')
                        driver_cam = cam1

            count += 1
            now = time.time()

            arr0 = cam0.capture_array('main')
            arr1 = cam1.capture_array('main')
            #dashboard_arr = driver_cam.capture_array('lores')

            if args.save:
                for (i, arr) in enumerate([arr0, arr1]):
                    streams[i].add_frame(arr)

            dashboard_arr = cv2.resize(arr0 if driver_cam is cam0 else arr1, (640, 480))
            source.putFrame(dashboard_arr)

            aprilDetector.detect(arr0)
            aprilDetector.detect(arr1)

            key = console.get_key()

            if now - reported > 1:
                fps = count / (now - reported)
                reported = now
                count = 0
                print(f'FPS: {fps}   ', end='\r')

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
        del console
        print('exiting')
        cam0.stop()
        cam1.stop()
        if streams:
            print('stop recordings')
            for stream in streams:
                stream.close()

def get_args():
    # pylint: disable=import-outside-toplevel
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('--res', default='640x480',
        help="camera resolution (default %(default)s)")
    parser.add_argument('--dec', type=int, default=2,
        help="AprilTag decimation (default %(default)s)")
    parser.add_argument('--threads', type=int, default=4,
        help="AprilTag threads (default %(default)s)")
    parser.add_argument('--fps', type=float, default=60.0,
        help="camera FPS (default %(default)s)")
    parser.add_argument('--cals',
        help="camera cal name (default matches resolution)")
    parser.add_argument('--cam0flip', action='store_true',
        help="flip camera 0 (180 rotation)")
    parser.add_argument('--cam1flip', action='store_true',
        help="flip camera 1 (180 rotation)")
    parser.add_argument('--save', action='store_true',
        help="record video (to SSD, defaults to H264 encoding)")
    parser.add_argument('--quality', type=int, default=None,
        help="record JPEG quality (50-100) (enables MJPEG encoding)")
    parser.add_argument('--serve', action='store_true',
        help="host NT instance for testing")

    args = parser.parse_args()
    if args.cals is None:
        args.cals = args.res
    args.res = tuple(int(x) for x in args.res.split('x'))
    return args

if __name__ == '__main__':
    main()
    
