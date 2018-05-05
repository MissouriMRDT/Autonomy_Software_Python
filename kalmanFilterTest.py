
import time
import math
import datetime
import os

from drivers.rovecomm import RoveComm
from drivers.navBoard import NavBoard
from algorithms.kalmanFilter import KalmanFilter

rovecomm_node = RoveComm()
navBoard = NavBoard(rovecomm_node)

time.sleep(1)

kalman = KalmanFilter(navBoard)
      
while True:

    heading, tiltCompensatedHeading = kalman.getHeadings()
    pitch = kalman.getPitch()
    roll = kalman.getRoll()
        
    if 1:			#Change to '0' to stop  showing the heading
        print ("HEADING %5.2f \33[1;37;40m tiltCompensatedHeading %5.2f" % (heading,tiltCompensatedHeading)),
        
    if 1:			#Change to '0' to stop
        print ("\033[1;31;40m pitch %5.2f  \033[1;35;40m roll %5.2f  " % (pitch, roll)),

    #print a new line
    print("")  


    #slow program down a bit, makes the output more readable
    time.sleep(0.1)
