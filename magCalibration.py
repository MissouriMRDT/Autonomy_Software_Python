import sys,signal,os
import time
import math
import json

import datetime

from drivers.navBoard import NavBoard
from drivers.rovecomm import RoveComm


def handle_ctrl_c(signal, frame):
    print(" ")
    print("magXmin = ",  magXmin)
    print("magYmin = ",  magYmin)
    print("magZmin = ",  magZmin)
    print("magXmax = ",  magXmax)
    print("magYmax = ",  magYmax)
    print("magZmax = ",  magZmax)
    
    calibration = {"min_x": magXmin, "max_x": magXmax, "min_y": magYmin, "max_y": magYmax, "min_z": magZmin, "max_z": magZmax}
    with open('mag_calibration.json', 'w') as calfile:
        json.dump(calibration, calfile)

    sys.exit(130) # 130 is standard exit code for ctrl-c

rovecomm_node = RoveComm()
navBoard = NavBoard(rovecomm_node)

time.sleep(1)

#This will capture exit when using Ctrl-C
signal.signal(signal.SIGINT, handle_ctrl_c)


a = datetime.datetime.now()


#Preload the variables used to keep track of the minimum and maximum values
magXmin = 32767
magYmin = 32767
magZmin = 32767
magXmax = -32767
magYmax = -32767
magZmax = -32767


    
while True:

    #Read magnetometer values
    MAGx, MAGy, MAGz = navBoard.magnetometerXYZ()
    
    
    if MAGx > magXmax:
        magXmax = MAGx
    if MAGy > magYmax:
        magYmax = MAGy
    if MAGz > magZmax:
        magZmax = MAGz

    if MAGx < magXmin:
        magXmin = MAGx
    if MAGy < magYmin:
        magYmin = MAGy
    if MAGz < magZmin:
        magZmin = MAGz

    print(" magXmin  %i  magYmin  %i  magZmin  %i  magXmax  %i  magYmax  %i  magZmax %i  ", magXmin, magYmin, magZmin, magXmax, magYmax, magZmax)

    #slow program down a bit, makes the output more readable
    time.sleep(0.03)
