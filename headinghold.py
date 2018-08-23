import sys

from algorithms.PIDcontroller import *
from algorithms.quaternion import Quaternion
from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard
from drivers.navBoard import NavBoard
import logging
import time

logger = logging.getLogger(__name__)
#kp=5, ki=.2 kd=0
pid = PIDcontroller(Kp=5.5, Ki=0.15, Kd=0, wraparound=360)
        
def headingHold(goal, quaternion_ref, drive, speed):

    # If close enough to goal, just go straight
    # Also if on a roll, go straight because we can't trust heading
    if abs(goal - quaternion_ref.heading) < 5 or abs(quaternion_ref.roll) > 10:
        print("Going straight for goal! Roll: %f" %(quaternion_ref.roll))
        drive.move(speed, 0)
    else:
        correction = pid.update(goal, quaternion_ref.heading)
        print("Correction : %f "% correction)
        clamp(correction, -180, 180)
        drive.move(speed,  correction)
    
if __name__ == "__main__":
    rovecomm_node = RoveComm()
    driveBoard = DriveBoard(rovecomm_node)
    navBoard = NavBoard(rovecomm_node)
    time.sleep(1) # Let the magnetometer lock
    quaternion = Quaternion(navBoard)
    
    try:
        goal = float(sys.argv[1])
        speed = float(sys.argv[2])
    except:
        print("Usage: python headinghold <angle> <speed> \
              \t<angle>: Degrees deviation from due north. Float between 0.0 and 360.0 \
              \n<speed>: Percentage of full speed to go")
        quit()
    
    print ("Starting heading hold routine. Goal: ", goal, " degrees")
    driveBoard.enable()
    prevheading = None
    while(True):
        heading = quaternion.heading
        if heading != prevheading:
            headingHold(goal, quaternion, driveBoard, speed)
            prevheading = heading
        print("\tGoal: %f \tHeading: %f" % (goal, quaternion.heading))
        time.sleep(0.1)
