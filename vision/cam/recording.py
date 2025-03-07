
import datetime as dt
import logging
from pathlib import Path
import time
import subprocess

VIDEO_DIR = Path('videos')

class VideoEncoder:
    # How soon to retry after a failure
    RETRY_INTERVAL = 5.0
    
    def __init__(self, num, fps=30, width=1456, height=1088, quality=None, debug=False):
        self.width = width
        self.height = height
        self.fps = fps
        self.num = num
        self.quality = quality # used with MJPEG encoder
        self.process = None
        self.debug = debug

        self.log = logging.getLogger(f"rec{num}")
        self.path = None
        self.process = None
        self.retry_time = time.monotonic()
        self.warned = False
        

    def start(self):
        # TODO: different file extension if quality set (i.e. mjpeg)?
        ts = dt.datetime.now().strftime('%Y%m%d-%H%M%S.mp4')
        self.path = VIDEO_DIR / f"cam{self.num}-{ts}"

        cmd = [
            'ffmpeg',
            '-y',  # Overwrite output file
            '-f', 'rawvideo',
            '-vcodec', 'rawvideo',
            '-s', f'{self.width}x{self.height}',
            '-pix_fmt', 'bgr24',  # OpenCV default format
            '-r', str(self.fps),
            '-i', '-',  # Input from pipe
        ]

        if self.quality is None: # H264 encoding
            cmd.extend([
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                # '-tune', 'zerolatency',

                '-crf', '20',  # Balance quality/performance
                # Range: 0-51 (lower = higher quality, larger file)
                # Sweet spots:
                # 17-18: Visually lossless
                # 23: Default, good quality
                # 25-28: Acceptable quality with good compression
                # 30+: Noticeable quality loss
                #
                # Note: when using 25 I was getting a file size
                # about 1/30 of the MJPEG version.
                
                '-g', '60', # keyframe interval (1 every 30 frames)
                '-bf', '0', # no B frames (reduces CPU usage)
                 
                '-threads', '3',
                ])
            # 2025-03-06 with crf 20, g 60, bf 0, threads 3, I'm getting
            # cpu usage around 50-60% (load average 3+).
            # With crf 25, and neither g/bf args, and threads 4 it was
            # higher.  File size is fine either way, though quite a bit
            # smaller with the latter options, but we probably care more
            # about cpu usage and quality than file size.
        else:
            cmd.extend([
                '-c:v', 'mjpeg',
                '-q:v', str(int(31 * (1 - self.quality/100))),  # Convert quality to FFmpeg scale
                ])

        cmd.append(self.path)
        if not self.debug:
            cmd.extend(['-v', 'quiet'])
        
        args = {} if self.debug else {"stdout": subprocess.DEVNULL, "stderr": subprocess.DEVNULL}
        self.process = subprocess.Popen(cmd, stdin=subprocess.PIPE, **args)
        
    def add_frame(self, frame):
        if self.process is None:
            if time.monotonic() > self.retry_time:
                self.width = frame.shape[1]
                self.height = frame.shape[0]
                self.start()

        if self.process is not None:
            try:
                self.process.stdin.write(frame.tobytes())
                self.warned = False # clear the flag so future warnings will appear
            except BrokenPipeError:
                if not self.warned:
                    self.warned = True
                    self.log.warn("recording failed, will retry")
                self.close()
                self.retry_time = time.monotonic() + self.RETRY_INTERVAL
        
    def close(self):
        if self.process:
            try:
                self.process.stdin.close()
                self.process.wait()
            except Exception as ex:
                self.log.exception("closing failed")
            self.process = None
