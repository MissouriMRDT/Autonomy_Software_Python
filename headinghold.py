import time
import hmc5883l as magnetometer
from motors import Motors
from servodriver import ServoDriver
from PIDcontroller import *

# For the interactive tester
import sys
import time

pid = PIDcontroller(Kp=1, Ki=0, Kd=0, wraparound=360)
        
def headinghold(goal, actual_heading, motors, speed):
    correction = pid.update(goal, actual_heading)
    clamp(correction, -180, 180)
    motors.move(speed,  correction)
    
if __name__ == "__main__":

    servos = ServoDriver()
    motor_ctl = Motors(servos)
    mag = magnetometer.hmc5883l(gauss = 1.3, declination = (-90,0))
    
    try:
        goal = float(sys.argv[1])
        speed = float(sys.argv[2])
    except:
        print("Usage: python headinghold <angle> <speed> \
              \t<angle>: Degrees deviation from due north. Float between 0.0 and 360.0 \
              \n<speed>: Percentage of full speed to go")
        quit()
    
    print "Starting heading hold routine. Goal: ", goal, " degrees"
    motor_ctl.enable()
    while(True):
        headinghold(goal, mag.heading(), motor_ctl, speed)
        
        time.sleep(0.01)
