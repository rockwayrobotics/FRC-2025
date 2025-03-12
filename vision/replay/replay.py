
import cv2
import numpy as np
import os
import pathlib
import sys

from cam.camcal import CALS
from cam.apriltag import AprilTagDetection
from .fake_ntcore import NetworkTableInstance

def main():
    args = get_args()
    filebase = pathlib.Path(args.file).stem
    # Playing video from file:
    cap = cv2.VideoCapture(args.file)
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    try:
        cal = CALS[args.cals]
    except KeyError:
        cal = CALS[args.res]

    nt = NetworkTableInstance.getDefault()
    aprilDetector = AprilTagDetection(args, nt, cal=cal)

    cv2.namedWindow("Video")
    runUntilFound = True
    while cap.isOpened():
        ret, frame = cap.read()
        cur_frame_number = cap.get(cv2.CAP_PROP_POS_FRAMES)
        print('** At frame #' + str(cur_frame_number))
        if not ret:
            break

        cv2.imshow("Video", frame)
        detected = aprilDetector.detect(frame)
        if detected or not runUntilFound:
            key = cv2.waitKey(0)
            if key & 0xff == ord('b'):
                cur_frame_number = cap.get(cv2.CAP_PROP_POS_FRAMES)

                prev_frame = cur_frame_number
                # jump back two because we will advance when we read
                if (cur_frame_number >= 2):
                    prev_frame -= 2

                cap.set(cv2.CAP_PROP_POS_FRAMES, prev_frame)
                runUntilFound = False
            elif key & 0xff == ord('n'):
                runUntilFound = False
            elif key & 0xff == ord('p'):
                runUntilFound = True
            elif key & 0xff == ord(' '):
                cv2.imwrite(f'{filebase}-frame-{int(cur_frame_number)}.png', frame)
            elif key & 0xff == ord('q'):
                break
        else:
            key = cv2.waitKey(10)
            if key & 0xff == ord('p'):
                runUntilFound = False
            elif key & 0xff == ord('q'):
                break
        

    # When everything done, release the capture
    cap.release()

def get_args():
    # pylint: disable=import-outside-toplevel
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', default=None)
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

    args = parser.parse_args()
    if args.file is None:
        print('Please specify a replay video')
        sys.exit(1)
    if args.cals is None:
        args.cals = args.res
    args.res = tuple(int(x) for x in args.res.split('x'))
    return args

if __name__ == '__main__':
    main()