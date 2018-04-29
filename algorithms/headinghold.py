import sys

from algorithms.PIDcontroller import *
from drivers.rovecomm import RoveComm
from drivers.mag.compass import Compass
from drivers.driveBoard import DriveBoard
import logging
import time

logger = logging.getLogger(__name__)
#kp=5, ki=.2 kd=0
pid = PIDcontroller(Kp=5.5, Ki=0.15, Kd=0, wraparound=360)
        
def headingHold(goal, actual_heading, drive, speed):
    correction = pid.update(goal, actual_heading)
    logger.debug("Correction : %f "% correction)
    clamp(correction, -180, 180)
    drive.move(speed,  correction)
    
if __name__ == "__main__":
    rovecomm_node = RoveComm()
    drive_ctl = DriveBoard(rovecomm_node)
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
    
    print ("Starting heading hold routine. Goal: ", goal, " degrees")
    prevheading = None
    while(True):
        heading = mag.heading()
        if heading != prevheading:
            headingHold(goal, heading, drive_ctl, speed)
            prevheading = heading
        print("\tGoal: %f \tHeading: %f" % (goal, mag.heading()))
        time.sleep(0.01)
