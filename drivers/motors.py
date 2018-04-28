import time

import drivers.servodriver


class Motors:

    # Calibration constant for the motor controller
    centerpoint = 94
    
    def __init__(self, servodriver, leftchannel = 0, rightchannel = 1):
        self.controller = servodriver
        self.left = leftchannel
        self.right = rightchannel
        
        # Safe-Start conditions
        self.enable()
        
    def __del__(self):
        self.disable()
        
    def move(self, speed, angle):
        """ Speed: -100 to 100
            Angle: -180 = turn in place left, 0 = straight, 180 = turn in place right """
        speedl = speedr = Motors.centerpoint + (speed * 90.0/100.0)
        if angle < 0:
            speedl = speedl + ((angle / 180.0) * 2 * speed )
        elif angle > 0: 
            speedr = speedr - ((angle / 180.0) * 2 * speed )
            
        self.controller.movejoint(self.left, _clamp(speedl, 0, 180) )        
        self.controller.movejoint(self.right, _clamp(speedr, 0, 180) )
        
    def disable(self):
        self.controller.controller.set_pwm(self.left, 0, off=True)
        self.controller.controller.set_pwm(self.right, 0, off=True)
        
    def enable(self):
        self.controller.movejoint(self.left, Motors.centerpoint)
        self.controller.movejoint(self.right, Motors.centerpoint)
        time.sleep(0.5)

def _clamp(n, minn, maxn):
    return max(min(maxn, n), minn)
        
# Interactive mode
import sys,tty,termios

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
    while(1):
        print("Waiting for key")
        k=inkey()
        if k == '\x1b': # Arrow Key
            k=inkey() # Arrow keys are thee characters
            k=inkey() # Clear out buffer
            speed = 100
            if k=='A':
                print("up")
                motors.move(speed, 0)
            elif k=='B':
                print("down")
                motors.move(-speed, 0)
            elif k=='C':
                print("right")
                speed = speed + 10
                motors.move(speed, 180)
            elif k=='D':
                print("left")
                speed = speed + 10
                motors.move(speed, -180)
        elif k == ' ': # Space
            print("space")
            motors.move(0,0)
        elif k == '\x03': # Ctrl-C
            motors.disable()
            quit()
        else:
            print("Unexpected key ", k)

if __name__=='__main__':
    servodrv = drivers.servodriver.ServoDriver()
    motors = Motors(servodrv)
    print("Starting motor tester")
    while True:
        get(motors)
    motors.disable()
