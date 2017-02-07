# Robotic arm driver for Lil' Shitter
# Author: Owen Chiaventone
# License: get yer grubby hands off it. How did you even get this code?

import time
import Adafruit_PCA9685

NUM_CHANNELS = 16

class ServoDriver:
    servo_min = 245.  # Min pulse length out of 4096. Corresponds to 1.00 ms
    servo_max = 492.  # Max pulse length out of 4096. Corresponds to 2.00 ms

    def __init__(self):
        self.controller = Adafruit_PCA9685.PCA9685()
        self.controller.set_pwm_freq(57) # Comes out to 60 hz when accounting for my unit's clock skew
                     # Probably hacky. Should look into how to make this more accurate.
    def movejoint(self, joint, angle):
        """ angle should be between 0 and 180, inclusive
            Joint should be between 0 and 5, inclusive"""
        self.controller.set_pwm(joint, 0, scale(angle, 0, 180, self.servo_min, self.servo_max))
        
    def moveall(self, angle_list):
        for joint, angle in enumerate(angle_list):
            self.movejoint(joint, angle)

    def reset(self):
        self.moveall([90, 90, 90, 90, 90, 90])

    def limp(self):
        for channel in range(0, NUM_CHANNELS):
            self.controller.set_pwm(channel, 0, off=True)

    def sweepall(self):
        for joint in range(0, NUM_CHANNELS):
            for angle in range(0, 180):
                self.movejoint(joint, angle)
                time.sleep(0.01)
            self.movejoint(joint, 90)
        self.limp()
            

def scale(value, in_min, in_max, out_min, out_max):
    """ Equivalent of Arduino's Map() function. Map means something different in python"""
    return int(((value - in_min) * ((out_max - out_min) / (in_max - in_min))) + out_min)
