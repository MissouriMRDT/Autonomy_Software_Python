import sys
import tty
import termios
import struct
import threading
import time

from drivers.rovecomm import RoveComm

SPEED_LIMIT = 300


def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


class DriveBoard:
    def __init__(self, rovecomm_node):
        
        self.rovecomm_node = rovecomm_node
        
        self._targetSpdLeft = 0
        self._targetSpdRight = 0
        self._is_enabled = False
        
        self.updateThread = threading.Thread(target=self._updateThreadFxn) 
        self.updateThread.setDaemon(True)  # Automatically kill the thread when the program finishes
        self.updateThread.start()
        
    def __del__(self):
        self.disable()
        
    def move(self, speed, angle):
        """ Speed: -100 to 100
        Angle: -180 = turn in place left, 0 = straight, 180 = turn in place right """
        
        # Map speed and angle to (-1000, +1000) for each wheel
        speed_left = speed_right = speed / 100.0
        if angle > 0:
            speed_right = speed_right * (1 + (angle / 180.0))
        elif angle < 0:
            speed_left = speed_left * (1 - (angle / 180.0))
        
        self._targetSpdLeft = int(clamp(1000.0 * speed_left, -SPEED_LIMIT, SPEED_LIMIT))
        self._targetSpdRight = int(clamp(1000.0 * speed_right, -SPEED_LIMIT, SPEED_LIMIT))

    def _updateThreadFxn(self):
        while 1:
            if self._is_enabled:
                assert(-1000 < self._targetSpdLeft < 1000)
                assert(-1000 < self._targetSpdRight < 1000)
                self.rovecomm_node.sendDriveCommand(self._targetSpdLeft, self._targetSpdRight)

            time.sleep(0.01)

    def disable(self):
        self.move(0, 0)
        self._is_enabled = False

    def enable(self):
        self._is_enabled = True

        
############################################
# Interactive testing mode
# ------------------------------------------


class _Getch:
    def __call__(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def get(motors):
    inkey = _Getch()
    prev = False
    while 1:
        print("Waiting for key")
        k = inkey()
        if k == '\x1b':  # Arrow Key
            k = inkey()  # Arrow keys are thee characters
            k = inkey()  # Clear out buffer
            speed = 10
            if k == 'A':
                print("up")
                motors.move(speed, 0)
            elif k == 'B':
                print("down")
                motors.move(-speed, 0)
            elif k == 'C':
                print("right")
                motors.move(speed, 180)
            elif k == 'D':
                print("left")
                motors.move(speed, -180)
        elif k == 'e':
            print("enable")
            motors.enable()
        elif k == ' ':  # Space
            print("space")
            motors.disable()
        elif k == '\x03':  # Ctrl-C
            motors.disable()
            quit()
        else:
            print("Unexpected key ", k)


if __name__ == '__main__':
    rovecomm_node = RoveComm()
    motors = DriveBoard(rovecomm_node)
    print("Starting motor tester")
    while True:
        get(motors)
    motors.disable()
