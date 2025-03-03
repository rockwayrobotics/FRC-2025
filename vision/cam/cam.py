#!/usr/bin/env python3

# TODO: camera calibration https://learnopencv.com/camera-calibration-using-opencv/
# see also https://github.com/tobybreckon/python-examples-cv/blob/master/calibrate_camera.py
# chessboard: https://docs.opencv.org/3.4/pattern.png

import datetime as dt
import io
import logging
import threading
import math
import os
import platform
import time
import traceback

try:
    from libcamera import Transform
    import picamera2
    pi = True
except ImportError:
    pi = False


degrees = lambda rad: rad * 180 / math.pi

class PiCam:
    def __init__(self, args, id, fps=30):
        self.cam = picamera2.Picamera2(id)
        self.id = id

        # Default exposure time is 22162 us
        # To get fast fps, need Exposure time lower, buffer_count > 1 (4 is
        # better than 2), and FrameDurationLimits maybe (slight difference)
        vid1 = self.cam.create_video_configuration(
            main=dict(size=SIZE, format='RGB888'),
            lores=dict(size=[320,240], format='RGB888'),
            controls=dict(ExposureTime=11081, FrameRate=args.fps,
                          FrameDurationLimits=(1,17000)), # FrameDurationLimits=(1, 5000)),
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

class MockCam:
    def __init__(self, args, id):
        self.id = id

        import cv2
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


if pi:
    Camera = PiCam
else:
    Camera = MockCam
    
