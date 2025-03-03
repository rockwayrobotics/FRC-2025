'''Camera classes'''

# Reference camera calibration https://learnopencv.com/camera-calibration-using-opencv/
# see also https://github.com/tobybreckon/python-examples-cv/blob/master/calibrate_camera.py
# chessboard: https://docs.opencv.org/3.4/pattern.png

# pylint: disable=missing-function-docstring,missing-class-docstring,no-member

import math
from pathlib import Path

import cv2

try:
    from libcamera import Transform
    import picamera2
    PI = True
except ImportError:
    PI = False


def degrees(rad):
    '''Convert radians to degrees.'''
    return rad * 180 / math.pi


class PiCam:
    '''Raspberry Pi camera (use MockCam if not on a Pi)'''
    def __init__(self, num, res, fps=30):
        self.cam = picamera2.Picamera2(num)
        self.num = num

        # Default exposure time is 22162 us
        # To get fast fps, need Exposure time lower, buffer_count > 1 (4 is
        # better than 2), and FrameDurationLimits maybe (slight difference)
        vid1 = self.cam.create_video_configuration(
            main={"size": res, "format": "RGB888"},
            lores={"size": [320, 240], "format": "RGB888"},
            controls={"ExposureTime": 11081, "FrameRate": fps,
                      "FrameDurationLimits": (1, 17000)},  # FrameDurationLimits=(1, 5000)),
            queue=False,
            buffer_count=4,
            transform=Transform(hflip=1, vflip=1),
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
    def __init__(self, num, res, fps=30):
        self.num = num

        imgpath = Path(__file__).parent.absolute() / f'cam{self.num}.jpg'
        image = cv2.imread(imgpath)
        self._main = cv2.resize(image, (res[0], res[1]))
        self._lores = cv2.resize(image, (320, 240))

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

if __name__ == '__main__':
    cam = Camera(0, (640, 480))
    cam.start()
    img = cam.capture_array()
    cam.stop()
    print(img.shape)
