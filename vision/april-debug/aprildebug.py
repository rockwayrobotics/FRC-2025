import cv2
import robotpy_apriltag as at

filenames = [
    'debug_clusters.pnm',
    'debug_output.pnm',
    'debug_preprocess.pnm',
    'debug_quads_fixed.pnm',
    'debug_samples.pnm',
    'debug_threshold.pnm',
    'debug_lines.ps',
    'debug_output.ps',
    'debug_quads.ps',
    'debug_quads_raw.pnm',
    'debug_segmentation.pnm'
]

def main():
    args = get_args()
    det = at.AprilTagDetector()
    det.addFamily('tag36h11') # default: bitsCorrected=2
    cfg = det.getConfig()
    cfg.debug = True
    cfg.quadDecimate = args.dec
    cfg.numThreads = args.threads
    cfg.decodeSharpening = 0.25 # margin jumps a lot with 1.0
    # cfg.quadSigma = 0.8
    det.setConfig(cfg)
    quadParams = det.getQuadThresholdParameters()
    quadParams.minClusterPixels = 10
    det.setQuadThresholdParameters(quadParams)

    arr = cv2.imread(args.file)
    img = cv2.cvtColor(arr, cv2.COLOR_RGB2GRAY)
    tags = det.detect(img)
    print(tags)
    cv2.namedWindow('Debug')

    index = 0
    while True:
        print(filenames[index])
        try:
            img = cv2.imread(filenames[index])
            cv2.imshow('Debug', img)
        except Exception as e: 
            print('Failed to show:', e)
        key = cv2.waitKey(0)
        if key & 0xff == ord('q'):
            break
        elif key & 0xff == ord('n'):
            index = (index + 1) % len(filenames)
        elif key & 0xff == ord('p'):
            index = (index - 1) % len(filenames)

    cv2.destroyAllWindows()


def get_args():
    # pylint: disable=import-outside-toplevel
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('-f', '--file', required=True)
    parser.add_argument('-d', '--debug', action='store_true')
    parser.add_argument('--res', default='640x480',
        help="camera resolution (default %(default)s)")
    parser.add_argument('--dec', type=int, default=2,
        help="AprilTag decimation (default %(default)s)")
    parser.add_argument('--threads', type=int, default=4,
        help="AprilTag threads (default %(default)s)")
    parser.add_argument('--cals',
        help="camera cal name (default matches resolution)")

    args = parser.parse_args()
    if args.cals is None:
        args.cals = args.res
    args.res = tuple(int(x) for x in args.res.split('x'))
    return args

if __name__ == '__main__':
    main()
    
