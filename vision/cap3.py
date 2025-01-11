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
import time

from keys import NonBlockingConsole

import cv2
import robotpy_apriltag as at

from ntcore import NetworkTableInstance, PubSubOptions
from wpimath.geometry import Transform3d, Translation3d, Rotation3d, Pose2d
from wpimath import units

from libcamera import Transform
import picamera2
from picamera2.encoders import JpegEncoder
from picamera2.outputs import FileOutput

degrees = lambda rad: rad * 180 / math.pi

PAGE = """\
<html>
<head>
<title>picamera2 MJPEG streaming demo</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" height="480" />
</body>
</html>
"""

class StreamingOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamingHandler(server.BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(301)
            self.send_header('Location', '/index.html')
            self.end_headers()
        elif self.path == '/index.html':
            content = PAGE.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(content))
            self.end_headers()
            self.wfile.write(content)
        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception as e:
                logging.warning(
                    'Removed streaming client %s: %s',
                    self.client_address, str(e))
        else:
            self.send_error(404)
            self.end_headers()


class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


# imx219 [3280x2464] (/base/soc/i2c0mux/i2c@1/imx219@10)
# 'SRGGB10_CSI2P' :  640x480  [206.65 fps - (1000, 752)/1280x960 crop]
#                   1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
#                   1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
#                   3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]
#        'SRGGB8' :  640x480  [206.65 fps - (1000, 752)/1280x960 crop]
#                   1640x1232 [41.85 fps - (0, 0)/3280x2464 crop]
#                   1920x1080 [47.57 fps - (680, 692)/1920x1080 crop]
#                   3280x2464 [21.19 fps - (0, 0)/3280x2464 crop]


def main():
    cam = picamera2.Picamera2()

    vid1 = cam.create_video_configuration(
        main=dict(size=SIZE),
        lores=dict(size=SIZE, format='YUV420'),
        controls=dict(FrameRate=args.fps), # FrameDurationLimits=(1, 5000)),
        queue=False,
        buffer_count=1,
        transform=Transform(hflip=1, vflip=1),
        )

    cam_config = vid1
    cam.align_configuration(cam_config)
    # print('Cam config:\n%s' % '\n'.join(f'{x:>15} = {y}' for x, y in cam_config.items()))
    cam.configure(cam_config)

    global output
    output = StreamingOutput()
    cam.start_recording(JpegEncoder(), FileOutput(output))
    # cam.start()

    class Server(threading.Thread):
        def run(self):
            address = ('', args.port)
            self.server = StreamingServer(address, StreamingHandler)
            print(f'MJPEG server running on port {args.port}')
            self.server.serve_forever()

        def close(self):
            self.server.shutdown()

    server = Server(daemon=True)
    server.start()
    # time.sleep(0.1)

    field = at.AprilTagFieldLayout("2025-reefscape.json")

    config = at.AprilTagPoseEstimator.Config(
        tagSize = units.inchesToMeters(6.5), # 16.51cm
        # images1
        # fx = 717.428307,
        # fy = 712.943769,
        # cx = 365.509738,
        # cy = 219.080481,

        # images2
        fx = 813.002665,
        fy = 814.367913,
        cx = 340.340811,
        cy = 248.727651,
        )
    estimator = at.AprilTagPoseEstimator(config)

    console = NonBlockingConsole()

    nt = NetworkTableInstance.getDefault()
    tagPose = nt.getFloatArrayTopic(
        "/Shuffleboard/Drivebase/Field2d/Robot"
    ).publish(PubSubOptions())  # Uses default options

    nt.startServer()

    try:
        det = at.AprilTagDetector()
        det.addFamily('tag36h11') # default: bitsCorrected=2
        cfg = det.getConfig()
        cfg.quadDecimate = args.dec
        cfg.numThreads = args.threads
        cfg.decodeSharpening = 0.25 # margin jumps a lot with 1.0
        # cfg.quadSigma = 0.8
        det.setConfig(cfg)
        # print(f'Apriltags:\n\t{cfg.decodeSharpening=} {cfg.numThreads=} {cfg.quadDecimate=}\n'
        #     f'\t{cfg.quadSigma=} {cfg.refineEdges=} {cfg.debug=}')
        # q = det.getQuadThresholdParameters()
        # print(f'Quad Threshold:\n\t{degrees(q.criticalAngle)=} {q.deglitch=} {q.maxLineFitMSE=}\n'
        #     f'\t{q.maxNumMaxima=} {q.minClusterPixels=} {q.minWhiteBlackDiff=}')

        now = start = time.time()
        reported = start
        count = 0
        missed = 0
        fps = 0
        found = False
        height = SIZE[1] * 2 // 3
        print('SIZE', SIZE, 'height', height)
        while now - start < args.time:
            # arr = cam.capture_array('lores')
            arr = cam.capture_array('main')
            # img = arr[:height,:]
            img = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
            tags = det.detect(img)
            count += 1
            now = time.time()
            if now - reported > 1:
                fps = count / (now - reported)
                reported = now
                count = 0

            if found != bool(tags):
                found = not found
                print()

            if not found:
                missed += 1
                print(f'\r{fps:3.0f} FPS: missed {missed}' + ' ' * 40, end='')

            if found:
                missed = 0
                tag = sorted(tags, key=lambda x: -x.getDecisionMargin())[0]
                c = tag.getCenter()
                tid = tag.getId()
                # pose = field.getTagPose(tid)
                tf = estimator.estimate(tag)
                # tf = tf + Transform3d(Translation3d(-2, 0, 2), Rotation3d())

                pose = [tf.z, -tf.x, -tf.rotation().y_degrees]
                tagPose.set(pose)
                # print(tf)
                tftext = f'T=({round(tf.x,2)},{round(tf.y,2)},{round(tf.z,2)}) R=({round(tf.rotation().x_degrees)},{round(tf.rotation().y_degrees)},{round(tf.rotation().z_degrees)})'

                hmat = '' # '[' + ', '.join(f'{x:.0f}' for x in x.getHomography()) + ']'
                margin = tag.getDecisionMargin()
                # print(f'\r{fps:3.0f} FPS: margin={margin:2.0f} @{c.x:3.0f},{c.y:3.0f} id={tid:2} {hmat}    ' % tags, end='')

                print(f'\r{fps:3.0f} FPS: m={margin:2.0f} @{c.x:3.0f},{c.y:3.0f} id={tid:2} {tftext}   ' % tags, end='')

            key = console.get_key()
            if key is not None:
                if key == ' ':
                    # Convert YUV420 to BGR (OpenCV's default color space)
                    # bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_I420)
                    ts = dt.datetime.now().strftime('%Y%m%d-%H%M%S')
                    path = f'images/{ts}-640x480.png'
                    cv2.imwrite(path, img)

                    # bgr = cv2.cvtColor(img)
                    # ts = dt.datetime.now().strftime('%Y%m%d-%H%M%S')
                    # path = f'images/luma-{ts}-640x480.png'
                    # cv2.imwrite(path, img)

                    print('captured image')

                elif key.lower() == 'q':
                    break

    finally:
        del console
        print()
        cam.stop()
        server.close()
        server.join()


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('--port', type=int, default=8000)
    parser.add_argument('--res', default='640x480')
    parser.add_argument('--dec', type=int, default=2)
    parser.add_argument('--threads', type=int, default=4)
    parser.add_argument('--fps', type=float, default=60.0)
    parser.add_argument('--time', type=float, default=10.0)

    args = parser.parse_args()
    SIZE = tuple(int(x) for x in args.res.split('x'))

    main()
