import sys
import termios
import tty
import select

class NonBlockingConsole:
    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setraw(self.fd)

    def get_key(self):
        # Returns None if no input, otherwise returns the key
        if select.select([sys.stdin], [], [], 0)[0] != []:
            key = sys.stdin.read(1)
            return key
        return None

    def __del__(self):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)


if __name__ == '__main__':
    # Example usage:
    console = NonBlockingConsole()
    try:
        while True:
            # Your image processing here

            key = console.get_key()
            if key == 'q':
                break
            elif key == 'r':
                print("r was pressed")
            elif key is not None:
                print(f"Got key: {repr(key)}")
    except KeyboardInterrupt:
        pass
