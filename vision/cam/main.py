#!/usr/bin/env python3

# pylint: disable=invalid-name,trailing-whitespace,missing-module-docstring
# pylint: disable=missing-function-docstring,missing-class-docstring,no-member
# pylint: disable=consider-using-with,too-few-public-methods

import datetime as dt
import io
import logging
import threading
import math
from netifaces import interfaces, ifaddresses, AF_INET
import os
import sys
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
from .camcal import CALS
from .apriltag import AprilTagDetection
from .keys import NonBlockingConsole
from .recording import VideoEncoder

STDOUT = sys.stdout.isatty()

def ip4_addresses():
    ip_list = []
    for interface in interfaces():
        for link in ifaddresses(interface).get(AF_INET, ()):
            ip_list.append(link['addr'])
    return ip_list

def main():
    args = get_args()

    fore_cam = cam.get_camera(cam.CAM1, args.res, fps=args.fps, flip=args.fore_flip)
    aft_cam = cam.get_camera(cam.CAM0, args.res, fps=args.fps, flip=args.aft_flip)
    try:
        chute_cam = cam.CV2Cam()
    except Exception as ex:
        chute_cam = cam.MockCam(3, args.res, fps=args.fps)

    if args.force == 'aft':
        driver_cam = aft_cam
        print('force aft cam')
    elif args.force == 'chute':
        driver_cam = chute_cam
        print('force chute cam')
    elif args.force == 'fore':
        driver_cam = fore_cam
        print('force fore cam')
    else:
        driver_cam = fore_cam

    nt = NetworkTableInstance.getDefault()
    if args.serve:
        print('starting test NT server')
        nt.startServer()
    else:
        nt.setServerTeam(8089)
        nt.startClient4("pi cam")

    source = CvSource("cam", VideoMode.PixelFormat.kBGR, args.res[0], args.res[1], int(args.fps))
    port = 5801
    mjpegServer = MjpegServer("mjpeg", port)
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

    pivotAngleSub = nt.getDoubleTopic("/AdvantageKit/Chute/PivotAngleRadians").subscribe(0)

    dsFMSAttachedSub = nt.getBooleanTopic("/AdvantageKit/DriverStation/FMSAttached").subscribe(False)
    dsEnabledSub = nt.getBooleanTopic("/AdvantageKit/DriverStation/Enabled").subscribe(False)

    isRecording = False

    mjpegUrls = [f"mjpg:http://10.80.89.11:{port}/?action=stream"]
    try:
        for address in ip4_addresses():
            mjpegUrl = f"mjpg:http://{address}:{port}/?action=stream"
            if mjpegUrl not in mjpegUrls:
                mjpegUrls += [mjpegUrl]
    except:
        pass
    mjpegTopic.set(mjpegUrls)

    fore_cam.start()
    aft_cam.start()
    chute_cam.start()

    # pick calibration, falling back on built-in cal for current resolution
    # TODO: support reading from a JSON file with lookup by camera id
    # if we can figure out a way of identifying individual cameras, and
    # possibly account for flip=True separately in case that affects the
    # values (or can we just "rotate" the centre/focal point values?)
    try:
        cal = CALS[args.cals]
    except KeyError:
        cal = CALS[args.res]

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
        force_save = False
        print('res', args.res)

        while True:
            currentCam = selectCameraSub.get()
            if args.force:
                pass
            elif currentCam == 'fore':
                if driver_cam is not fore_cam:
                    print('selecting fore cam')
                driver_cam = fore_cam
            elif currentCam == 'aft':
                if driver_cam is not aft_cam:
                    print('selecting aft cam')
                driver_cam = aft_cam
            elif currentCam == 'chute':
                if driver_cam is not chute_cam:
                    print('selecting chute cam')
                driver_cam = chute_cam
            elif currentCam == 'auto':
                leftVelocity = leftVelocitySub.get()
                rightVelocity = rightVelocitySub.get()
                deadbandThreshold = camAutoDeadbandSub.get()
                #print(f'deadbandThreshold={deadbandThreshold}', end = '\r\n')

                if abs(leftVelocity) > deadbandThreshold and abs(rightVelocity) > deadbandThreshold:
                    if leftVelocity > 0 and rightVelocity > 0:
                        if driver_cam is not fore_cam:
                            print('selecting fore cam')
                        driver_cam = fore_cam
                    elif leftVelocity < 0 and rightVelocity < 0:
                        if driver_cam is not aft_cam:
                            print('selecting aft cam')
                        driver_cam = aft_cam

            count += 1
            now = time.time()

            arr0 = fore_cam.capture_array('main')
            arr1 = aft_cam.capture_array('main')
            #dashboard_arr = driver_cam.capture_array('lores')

            if args.save:
                fmsAttached = dsFMSAttachedSub.get()
                enabled = dsEnabledSub.get()
                shouldRecord = fmsAttached or enabled or force_save
                if isRecording and not shouldRecord:
                    # Stop recording since the FMS just got detached
                    for i in [0, 1]:
                        streams[i].close()
                    isRecording = False
                elif shouldRecord:
                    isRecording = True

                if isRecording:
                    for (i, arr) in enumerate([arr0, arr1]):
                        streams[i].add_frame(arr)

            if driver_cam is chute_cam:
                dashboard_arr = chute_cam.capture_array()
                if pivotAngleSub.get() < 0:
                    dashboard_arr = cv2.rotate(dashboard_arr, cv2.ROTATE_180)
            else:
                dashboard_arr = cv2.resize(arr0 if driver_cam is fore_cam else arr1, (320, 240))
            source.putFrame(dashboard_arr)

            if not fore_cam.fake:
                aprilDetector.detect(arr0)
            if not aft_cam.fake:
                aprilDetector.detect(arr1)

            key = console.get_key()

            if now - reported > 1:
                fps = count / (now - reported)
                reported = now
                count = 0
                if STDOUT:
                    print(f'FPS: {fps}   ', end='\r')

            if key is not None:
                if key.lower() == 'q':
                    break
                elif key.lower() == 'c':
                    if driver_cam == fore_cam:
                        driver_cam = aft_cam
                        selectCameraPub.set("aft")
                    elif driver_cam == aft_cam:
                        driver_cam = chute_cam
                        selectCameraPub.set("chute")
                    else:
                        driver_cam = fore_cam
                        selectCameraPub.set("fore")
                    print(f'Switched to camera {driver_cam.num}')
                elif key == 'F':
                    force_save = True

    finally:
        del console
        print('exiting')
        fore_cam.stop()
        aft_cam.stop()
        chute_cam.stop()
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
    parser.add_argument('--aft-flip', action='store_true',
        help="flip aft camera (180 rotation)")
    parser.add_argument('--fore-flip', action='store_true',
        help="flip fore camera (180 rotation)")
    parser.add_argument('--save', action='store_true',
        help="record video (to SSD, defaults to H264 encoding)")
    parser.add_argument('--quality', type=int, default=None,
        help="record JPEG quality (50-100) (enables MJPEG encoding)")
    parser.add_argument('--serve', action='store_true',
        help="host NT instance for testing")
    parser.add_argument('--force', default='',
        help="force camera for testing (fore/aft/chute)")

    args = parser.parse_args()
    if args.cals is None:
        args.cals = args.res
    args.res = tuple(int(x) for x in args.res.split('x'))
    return args

if __name__ == '__main__':
    main()
    
