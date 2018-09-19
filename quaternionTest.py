import time
import math
import datetime
import os

from drivers.rovecomm import RoveComm
from drivers.navBoard import NavBoard
from algorithms.quaternion import Quaternion

rovecomm_node = RoveComm()
navBoard = NavBoard(rovecomm_node)

time.sleep(1)

fusion = Quaternion(navBoard)
      
while True:
        
    if 1:			#Change to '0' to stop  showing the heading
        print ("HEADING: %5.2f,  TRUEHEADING: %5.2f" % (fusion.heading, fusion.trueHeading)),
        
    if 1:			#Change to '0' to stop
        print (" pitch %5.2f, roll %5.2f  " % (fusion.pitch, fusion.roll)),

    #print a new line
    print("")


    #slow program down a bit, makes the output more readable
    time.sleep(0.1)
