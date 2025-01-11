import asyncio
import robotpy_apriltag as at
import cv2
import logging
import numpy as np
import platform
import sys
import threading
import time
import traceback

from .cam import Cam

vlog = logging.getLogger('vision')

class Processor:
    def __init__(self, shutdown):
        self.shutdown = shutdown

        self.det = at.AprilTagDetector()
        self.det.addFamily('tag36h11', bitsCorrected=0)
        # AprilTagDetector has a config that could be modified here
        # cfg = det.getConfg()
        # cfg.quadDecimate = 2.0 # default
        # cfg.decodeSharpening = 0.25 # default
        # cfg.quadSigma = 0.0 # default
        # det.setConfig(cfg)

        tagSize = 0.1651 # 6.5 in in meters
        cameraConfig = at.AprilTagPoseEstimator.Config(tagSize, 1, 1, 320, 320)
        self.estimator = at.AprilTagPoseEstimator(cameraConfig)
        self.field = at.AprilTagFieldLayout.loadField(at.AprilTagField.k2024Crescendo) # robotpy_apriltag doesn't have the 2025 field yet
        # even though it's been added to https://github.com/wpilibsuite/allwpilib/blob/main/apriltag/src/main/native/resources/edu/wpi/first/apriltag/2025-reefscape.json

        # If we actually want to draw on the image
        self.draw_tags = True

        # If we want to show the image in a window
        self.render_window = platform.machine() != 'aarch64' or sys.platform != 'darwin'

    def do_apriltag(self, lores, imgout):
        tags = self.det.detect(lores)
        if bool(tags):
            for (i, tag) in enumerate(sorted(tags, key=lambda x: x.getDecisionMargin())):
                c = tag.getCenter()
                x = int(c.x)
                y = int(c.y)
                tid = tag.getId()
                transform3d = self.estimator.estimate(tag)
                pose = self.field.getTagPose(tid)
                print(transform3d, pose)

                if self.draw_tags: 
                    # Draw a circle on the output image in the center of the tag
                    cv2.circle(imgout, (x, y), 5, (40, 0, 255), -1)

                    # Define the corners of the tag (assuming a 2x2 tag for simplicity)
                    tc = np.array([[-1, -1, 1], [1, -1, 1], [1, 1, 1], [-1, 1, 1]])

                    # Project the corners into the image plane
                    ic = (tag.getHomographyMatrix() @ tc.T).T

                    # Normalize the points
                    ic2 = (ic[:, :2] / ic[:, 2][:, np.newaxis]).astype(np.int32)
            
                    # Draw the rectangle
                    for i in range(4):
                        pt1 = tuple(ic2[i % 4])
                        pt2 = tuple(ic2[(i + 1) % 4])
                        cv2.line(imgout, pt1, pt2, (210, 30, 150), 4)

                    cv2.putText(imgout, f'{tag.getId()}', tuple(ic2[1] + [-4, 0]), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (128, 255, 128), 3, cv2.LINE_AA)
        return imgout

    def run(self, cam):
        base = time.monotonic()
        done = self.shutdown.is_set # local var for faster access
        while not done():
            t0 = time.monotonic()
            imain = cam.get_main()
            if imain is None:
                continue
            ilores = cam.get_lores()
            if ilores is None:
                continue

            t1 = time.monotonic()
            out = self.do_apriltag(ilores, imain)
            
            if self.render_window:
                cv2.imshow('Camera', out)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            now = time.monotonic()
            if now - base >= 2.5:
                base = now
                print(f' t={now-t1:.3f}s t={now-t0:.3f}s')

        vlog.debug('exiting run')

def run_vision(shutdown):
    cam = Cam()
    try:
        p = Processor(shutdown)
        p.run(cam)
    except Exception:
        traceback.print_exc()
    finally:
        cam.stop()

def run():
    try:
        shutdown = threading.Event()
        run_vision(shutdown)
        #await asyncio.to_thread(run_vision, shutdown)
    except asyncio.CancelledError:
        vlog.debug('cancelled')
    except Exception as ex:
        vlog.exception('run_vision failed')
    finally:
        shutdown.set()
