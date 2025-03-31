import argparse
import asyncio
import datetime as dt
import logging
import sys

from .logs import LOG_DIR
from .main import TofMain

DEFAULT_TOF_ADDRESS = 0x29
DEFAULT_TIMING = 20
DEFAULT_INTER = 25

def get_args():
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--debug", action="store_true")
    parser.add_argument("--debug-cd", action="store_true")
    parser.add_argument("--roi", default="16,16",
        help="region of interest width,height (default %(default)s)")
    parser.add_argument(
        "-t", "--timing", type=int, default=DEFAULT_TIMING,
        help="timing budget in ms (default %(default)s)")
    parser.add_argument(
        "-i", "--inter", type=int, default=DEFAULT_INTER,
        help="inter-measurement period in ms (default %(default)s)")
    parser.add_argument("--serve", action="store_true",
        help="serve NT instance (for testing)")
    parser.add_argument("--stdout", action="store_true")
    parser.add_argument("--slope", type=float, default=400,
        help="slope threshold (for cd1 and cd2) (default %(default)s)")
    parser.add_argument("--speed", type=float, default=450,
        help="fake speed (mm/s) (for testing, default %(default)s)")
    parser.add_argument("--log-cycle-time", type=int, default=30,
        help="Cycle log files after X minutes (default %(default)s, 0=off)")
    parser.add_argument("--tof-address", type=lambda x: int(x, 0), default=DEFAULT_TOF_ADDRESS,
        help="TOF address (default 0x%(default)02x)")

    args = parser.parse_args()
    args.roi = tuple(int(x) for x in args.roi.split(","))
    # Region of interest is width, height, values must be within 4-16 (inclusive)
    if args.roi[0] < 4 or args.roi[1] < 4:
        print("ROI width and height must be at least 4")
        sys.exit(0)

    return args


async def async_main(args):
    # need loop before we can initialize
    tof = TofMain(args)
    await tof.run()


def main():
    args = get_args()

    filename = LOG_DIR / f"tof-{dt.datetime.now().strftime('%Y%m%d-%H%M%S.log')}"
    handlers = [logging.FileHandler(filename, encoding="utf-8")]
    if args.stdout or os.environ.get('INVOCATION_ID') is not None:
        handlers.append(logging.StreamHandler(sys.stdout))
    level = logging.DEBUG if args.debug else logging.INFO
    logging.basicConfig(level=level,
        format="%(asctime)s.%(msecs)03d:%(levelname)5s:%(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=handlers
    )

    logging.getLogger('cd').setLevel(logging.DEBUG if args.debug_cd else logging.INFO)

    try:
        asyncio.run(async_main(args))
    finally:
        logging.info('main exiting')


if __name__ == '__main__':
    main()
