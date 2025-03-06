'''Camera classes'''

# Reference camera calibration https://learnopencv.com/camera-calibration-using-opencv/
# see also https://github.com/tobybreckon/python-examples-cv/blob/master/calibrate_camera.py
# chessboard: https://docs.opencv.org/3.4/pattern.png

# pylint: disable=missing-function-docstring,missing-class-docstring,no-member

import logging
import math
from pathlib import Path

import cv2
import numpy as np

try:
    from libcamera import Transform
    import picamera2
    PI = True
except ImportError:
    PI = False


def degrees(rad):
    '''Convert radians to degrees.'''
    return rad * 180 / math.pi


class CamCal:
    def __init__(self, fx, fy, cx, cy):
        self._array = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1], np.float64).reshape((3, 3))
        self._value = { "fx":fx, "fy":fy, "cx":cx, "cy": cy }
    def as_array(self):
        return self._array
    def as_dict(self):
        return self._value

CALS = {
    # images2 640x480
    '640x480': CamCal(fx=813.002665, fy=814.367913, cx=340.340811, cy=248.727651),

    # images4 1456x1088
    '1456x1088': CamCal(fx=1757.669488, fy=1762.782233, cx=736.690867, cy=557.428635),
}

class PiCam:
    '''Raspberry Pi camera (use MockCam if not on a Pi)'''
    def __init__(self, num, res, fps=30, flip=False):
        self.cam = picamera2.Picamera2(num)
        self.num = num

        # Default exposure time is 22162 us
        # To get fast fps, need Exposure time lower, buffer_count > 1 (4 is
        # better than 2), and FrameDurationLimits maybe (slight difference)
        vid1 = self.cam.create_video_configuration(
            main={"size": res, "format": "RGB888"},
            # lores={"size": [320, 240], "format": "RGB888"},
            controls={"ExposureTime": 11081, "FrameRate": fps,
                      "FrameDurationLimits": (1, 17000)},  # FrameDurationLimits=(1, 5000)),
            queue=False,
            buffer_count=4,
            transform=Transform(hflip=int(flip), vflip=int(flip)),
            )

        cam_config = vid1
        self.cam.align_configuration(cam_config)
        # print('Cam config:\n%s' % '\n'.join(f'{x:>15} = {y}' for x, y in cam_config.items()))
        self.cam.configure(cam_config)

    def capture_array(self, config='main'):
        '''Return latest frame as a numpy array. May block if frame not ready.'''
        return self.cam.capture_array(config)

    def stop(self):
        '''Stop the camera subsystem from capturing frames.'''
        self.cam.stop()

    def start(self):
        '''Start the camera capturing frames.'''
        self.cam.start()


class MockCam:
    # pylint: disable=unused-argument
    def __init__(self, num, res, fps=30, flip=False):
        self.num = num

        imgpath = Path(__file__).parent.absolute() / f'cam{self.num}.jpg'
        image = cv2.imread(imgpath)
        if flip:
            image = cv2.rotate(image, cv2.ROTATE_180)
        self._main = cv2.resize(image, (res[0], res[1]))
        self._lores = cv2.resize(image, (320, 240))

        print(f'created MockCam {num}')

    def capture_array(self, config='main'):
        return self._main if config == 'main' else self._lores

    def stop(self):
        pass

    def start(self):
        pass


if PI:
    Camera = PiCam
else:
    Camera = MockCam

# cam0 = cam.get_camera(0, args.res, fps=args.fps, flip=args.cam0flip)
def get_camera(num, res=(640, 480), fps=30, flip=False):
    try:
        cam = PiCam(num, res, fps=fps, flip=flip)
    except Exception as ex:
        logging.error("unable to open PiCam %s", num)
        cam = MockCam(num, res, fps=fps, flip=flip)
    return cam
                    

if __name__ == '__main__':
    cam = Camera(0, (640, 480))
    cam.start()
    img = cam.capture_array()
    cam.stop()
    print(img.shape)
