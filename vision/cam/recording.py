
import socketserver
from http import server
import threading
import subprocess

class VideoEncoder:
    def __init__(self, path, fps=30, width=1456, height=1088, quality=None, debug=False):
        self.width = width
        self.height = height
        self.fps = fps
        self.path = path
        self.quality = quality # used with MJPEG encoder
        self.process = None
        self.debug = debug
        
    def start(self):
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
            self.width = frame.shape[1]
            self.height = frame.shape[0]
            self.start()
            
        self.process.stdin.write(frame.tobytes())
        
    def close(self):
        if self.process:
            self.process.stdin.close()
            self.process.wait()
