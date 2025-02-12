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

is_pi = False
try:
    from libcamera import Transform
    import picamera2
    from picamera2.encoders import JpegEncoder
    from picamera2.outputs import FileOutput
    is_pi = True
except ImportError:
    pass

degrees = lambda rad: rad * 180 / math.pi

PAGE = """\
<html>
<head>
<title>picamera2 MJPEG streaming demo</title>
</head>
<body>
<h1>Picamera2 MJPEG Streaming Demo</h1>
<img src="stream.mjpg" width="640" />
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

class PiCam:
    def __init__(self, args):
        self.cam = picamera2.Picamera2()

        vid1 = self.cam.create_video_configuration(
            main=dict(size=SIZE, format='RGB888'),
            lores=dict(size=SIZE, format='YUV420'),
            controls=dict(FrameRate=args.fps), # FrameDurationLimits=(1, 5000)),
            queue=False,
            buffer_count=1,
            transform=Transform(hflip=1, vflip=1),
            )

        cam_config = vid1
        self.cam.align_configuration(cam_config)
        # print('Cam config:\n%s' % '\n'.join(f'{x:>15} = {y}' for x, y in cam_config.items()))
        self.cam.configure(cam_config)

    def setup_output(self, output):
        self.cam.start_recording(JpegEncoder(), FileOutput(output))
        # self.cam.start()

    def capture_array(self, config):
        return self.cam.capture_array(config)

    def stop(self):
        self.cam.stop()

class CV2Cam:
    def __init__(self, args):
        self.cam = cv2.VideoCapture(0)
        self.cam.set(cv2.CAP_PROP_FPS, args.fps)
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, SIZE[0])
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, SIZE[1])

    def setup_output(self, output):
        self.output = output

    def capture_array(self, config):
        for i in range(5):
            ret, frame = self.cam.read()
            if ret:
                encode_result, encode_frame = cv2.imencode('.jpg', frame)
                if encode_result:
                    self.output.write(encode_frame)
                if config == 'lores':
                    return cv2.cvtColor(frame, cv2.COLOR_RGB2YUV_YV12)
                else:
                    return frame
        raise Exception("Unable to read from camera after 5 attempts")

    def stop(self):
        self.cam.release()



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

    # Jeremy's Mac @640x480
    'jeremy-mac-640x480': dict(
        fx = 801.552718,
        fy = 782.872041,
        cx = 268.012054,
        cy = 125.630105,
    ),

    'jeremy-mac-1280x720': dict(
        fx = 895.628914,
        fy = 899.300743,
        cx = 655.965312,
        cy = 344.614969,
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
    if is_pi:
        cam = PiCam(args)
    else:
        cam = CV2Cam(args)

    global output
    output = StreamingOutput()
    cam.setup_output(output)

    nt = NetworkTableInstance.getDefault()
    nt.setServerTeam(8089)
    nt.startClient4("pi")

    source = CvSource("cam", VideoMode.PixelFormat.kBGR, SIZE[0], SIZE[1], int(args.fps))
    mjpegServer = MjpegServer("mjpeg", 8081)
    mjpegServer.setSource(source)
    mjpegTopic = nt.getStringArrayTopic("/CameraPublisher/PiCam/streams").publish(PubSubOptions())
    mjpegTopic.set(["mjpg:http://10.80.89.11:8081/?action=stream"])
    # mjpegServer = CameraServer.startAutomaticCapture(source)
    # print(mjpegServer.getPort())

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
        **CALS.get(args.cals, CALS.get('640x480'))
        )
    estimator = at.AprilTagPoseEstimator(config)

    console = NonBlockingConsole()

    tagPose = nt.getFloatArrayTopic(
        "/Shuffleboard/Drivebase/Field2d/Robot"
    ).publish(PubSubOptions())  # Uses default options
    tagPose2 = nt.getFloatArrayTopic(
        "/Shuffleboard/Drivebase/Field2d/Robot2"
    ).publish(PubSubOptions())  # Uses default options
    ambiguityTopic = nt.getFloatTopic(
        "/Shuffleboard/Drivebase/Field2d/Ambiguity"
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
            s = time.monotonic()
            img = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
            elapsed = time.monotonic() - s
            cv2.rectangle(arr, (10, 10), (110, 110), (0, 0, 255), -1)
            source.putFrame(arr)
            print(f'Elapsed {round(elapsed * 1000,3)} ms')
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
                absolutePose = field.getTagPose(tid)
                # tf = estimator.estimate(tag)
                poseEstimate = estimator.estimateOrthogonalIteration(tag, 50)
                ambiguity = poseEstimate.getAmbiguity()
                pose1 = cam2Field(absolutePose, poseEstimate.pose1)
                pose2 = cam2Field(absolutePose, poseEstimate.pose2)
                print(f'\rAmbiguity={ambiguity}')
                ambiguityTopic.set(ambiguity * 100)
                if pose1 is not None:
                    print(f'\r{poseEstimate.error1} Pose1={round(pose1.x,2)},{round(pose1.y,2)},{round(pose1.rotation().degrees())}')
                    tagPose.set([pose1.x, pose1.y, pose1.rotation().radians()])
                if pose2 is not None and ambiguity > 0:
                    print(f'\r{poseEstimate.error2} Pose2={round(pose2.x,2)},{round(pose2.y,2)},{round(pose2.rotation().degrees())}')
                    tagPose2.set([pose2.x, pose2.y, pose2.rotation().radians()])
                if pose1 is not None and pose2 is not None:
                    print(f'\rDistance: {pose1.translation().distance(pose2.translation())}')

                tf = poseEstimate.pose1
                rp = pose1

                pose = [rp.x, rp.y, rp.rotation().radians()]

                # This is the pose from camera which could be sent as a tag for debugging
                # pose = [tf.z, -tf.x, -tf.rotation().y_degrees]
                # tagPose.set(pose)
                # print(tf)
                tftext = f'T=({round(tf.x,2)},{round(tf.y,2)},{round(tf.z,2)}) R=({round(tf.rotation().x_degrees)},{round(tf.rotation().y_degrees)},{round(tf.rotation().z_degrees)})'
                # rptext = f'T=({round(rp.x,2)},{round(rp.y,2)},{round(rp.z,2)}) R=({round(rp.rotation().x_degrees)},{round(rp.rotation().y_degrees)},{round(rp.rotation().z_degrees)})'
                rptext = f'T=({round(rp.x,2)},{round(rp.y,2)}) R=({round(rp.rotation().degrees())})'

                hmat = '' # '[' + ', '.join(f'{x:.0f}' for x in x.getHomography()) + ']'
                margin = tag.getDecisionMargin()
                # print(f'\r{fps:3.0f} FPS: margin={margin:2.0f} @{c.x:3.0f},{c.y:3.0f} id={tid:2} {hmat}    ' % tags, end='')

                # print(f'\r{fps:3.0f} FPS: m={margin:2.0f} @{c.x:3.0f},{c.y:3.0f} id={tid:2} {tftext} {rptext}   ' % tags, end='')

            key = console.get_key()
            if key is not None:
                if key == ' ':
                    # Convert YUV420 to BGR (OpenCV's default color space)
                    # bgr = cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_I420)
                    ts = dt.datetime.now().strftime('%Y%m%d-%H%M%S')
                    try:
                        os.mkdir('images')
                    except FileExistsError:
                        pass
                    path = f'images/{ts}-{args.res}.png'
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
    parser.add_argument('--time', type=float, default=10000.0)
    parser.add_argument('--cals')

    args = parser.parse_args()
    if args.cals is None:
        args.cals = args.res
    SIZE = tuple(int(x) for x in args.res.split('x'))

    main()
