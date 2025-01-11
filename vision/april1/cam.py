import platform
import sys

is_pi = platform.machine() == 'aarch64' and sys.platform != 'darwin'
if is_pi:
    import libcamera
    from picamera2 import Picamera2
    class CamPi:
        def __init__(self, fps, size):
            self.cam = Picamera2(0)
            cfg = self.cam.create_video_configuration(
                controls=dict(FrameRate=fps),
                main=dict(size=size, format='RGB888'),
                lores=dict(size=size, format='YUV420'),
            )
            cfg['transform'] = libcamera.Transform(hflip=1, vflip=1)
            print(cfg)
            self.cam.configure(cfg)
            self.cam.start()
            pass

        def get_main(self):
            return self.cam.capture_array('main')

        def get_lores(self):
            return self.cam.capture_array('lores')

        def stop(self):
            self.cam.stop()
else:
    import cv2
    class CamCV2:
        def __init__(self, fps, size):
            self.last_frame = None
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FPS, fps)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            pass

        def get_main(self):
            ret, frame = self.cap.read()
            # TODO handle errors
            if ret:
                self.last_frame = frame
            return frame

        def get_lores(self):
            if self.last_frame is None:
                self.get_main()
            
            if self.last_frame is None:
                return None

            # YV12 is stored in separate regions, so it actually
            # has dimensions roughly w x (1.5h). However, we only
            # want the Y region, which we are using as our grayscale
            # approximation.
            # We could simply convert with RGB2GRAY but that's not
            # as fast on the Pi.
            yuv = cv2.cvtColor(self.last_frame, cv2.COLOR_RGB2YUV_YV12)
            yuv = yuv[:self.height,:]

            return yuv

        def stop(self):
            self.cap.release()

class Cam:
    def __init__(self, fps=60.0, size=(640,480)):
        if is_pi:
            self.impl = CamPi(fps, size)
        else:
            self.impl = CamCV2(fps, size)
    
    def get_main(self):
        return self.impl.get_main()

    def get_lores(self):
        return self.impl.get_lores()

    def stop(self):
        return self.impl.stop()
