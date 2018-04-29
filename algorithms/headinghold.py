import sys

from algorithms.PIDcontroller import *
from drivers.Magnetometer import Compass
from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard

pid = PIDcontroller(Kp=8, Ki=0, Kd=0, wraparound=360)
        
def headinghold(goal, actual_heading, motors, speed):
    correction = pid.update(goal, actual_heading)
    clamp(correction, -180, 180)
    motors.move(speed,  correction)
    
if __name__ == "__main__":
    rovecomm_node = RoveComm()
    drive_ctl = DriveBoard(rovecomm_node)
    mag = Compass(rovecomm_node)
    
    try:
        goal = float(sys.argv[1])
        speed = float(sys.argv[2])
    except:
        print("Usage: python headinghold <angle> <speed> \
              \t<angle>: Degrees deviation from due north. Float between 0.0 and 360.0 \
              \n<speed>: Percentage of full speed to go")
        quit()
    
    print ("Starting heading hold routine. Goal: ", goal, " degrees")
    while(True):
        headinghold(goal, mag.heading(), drive_ctl, speed)
        
        time.sleep(0.01)
