import sys

from algorithms.PIDcontroller import *
from drivers.rovecomm import RoveComm
from drivers.Magnetometer import Compass
from drivers.motorsRoveComm import Motors
import time

pid = PIDcontroller(Kp=10, Ki=2, Kd=0, wraparound=360)
        
def headinghold(goal, actual_heading, motors, speed):
    correction = pid.update(goal, actual_heading)
    print("Correction : %f "% correction)
    clamp(correction, -180, 180)
    motors.move(speed,  correction)
    
if __name__ == "__main__":
    rovecomm_node = RoveComm()
    motor_ctl = Motors()
    mag = Compass(rovecomm_node)
    time.sleep(0.5) # Let the magnetometer lock
    
    try:
        goal = float(sys.argv[1])
        speed = float(sys.argv[2])
    except:
        print("Usage: python headinghold <angle> <speed> \
              \t<angle>: Degrees deviation from due north. Float between 0.0 and 360.0 \
              \n<speed>: Percentage of full speed to go")
        quit()
    
    print "Starting heading hold routine. Goal: ", goal, " degrees"
    prevheading = None
    while(True):
        heading = mag.heading()
        if heading != prevheading:
            headinghold(goal, heading, motor_ctl, speed)
            prevheading = heading
        print("\tGoal: %f \tHeading: %f" % (goal, mag.heading()))
        time.sleep(0.01)
