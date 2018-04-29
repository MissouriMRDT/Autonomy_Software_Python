# Author: Sarah Dlouhy

from sweeppy import Sweep as sweep
from sweeppy import Sample
import math
import itertools

from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard


class LiDAR:

    # closest rover gets to object before deciding what to do
    MIN_SAFE_DIST = 1  # meters because i can
    Y_TOL = .05  # meters
    MAX_Y_UP = 1.00  # meters
    MIN_Y_DOWN = .75  # meters, separate to change on the fly if needed

    # will mess with the sweeping when main part works

    def __init__(self):
        self.lidar = "/dev/ttyUSB0"
        self.scan = None
        self.drive = DriveBoard(RoveComm())
        self.discont = False
        # self.servo = "i'll figure this out later"

    def connection(self):
        self.sweep.__enter__()
        self.sweep.set_motor_speed(10)
        self.sweep.set_sample_rate(1000)
        self.sweep.start_scanning()


        # sets servo to new direction if angle of point from LiDAR is 0 degrees
       # def servo_pos(self):
           # if self.th1 == 0:
             #   while self.th2 < 180:
            #        self.th2 += .5
            #    if self.th2 == 180:
           #        while self.th2 > 0:
           #             self.th2 -= .5
           # print("servo angle is:", self.th2)
        #return self.th2

    def jumps(self):
        for p in self.slope_arr:
            while p != 0:
                y_diff = self.slope_arr[p] - self.slope_arr[p - 1]
                if math.fabs(y_diff) > self.Y_TOL:
                    if self.MIN_Y_DOWN > y_diff < 0:
                        self.drive.move(15, 15)
                        self.discont = True
                    elif self.MAX_Y_UP < y_diff > 0:
                        self.drive.move(15, -15)
                        self.discont = True
        return self.discont


    # essentially determines what data goes into the array to compare slopes
    # data that gets too close does not make it
    def get_data(self):
        lidar_arr = []
        for scans in itertools.islice(self.sweep.get_scans(), 1):
            for sample in scans.samples:
                th1 = sample.angle / 1000.00  # degrees
                # th2     = servo_pos()
                tot_mag = sample.distace / 100.000
                y_mag = tot_mag * math.sin(math.radians(th1))
                while 190.00 < th1 < 340.00:
                    if tot_mag > self.MIN_SAFE_DIST:
                        lidar_arr.append(tot_mag, y_mag, th1)
                        self.discont = self.jumps()
                    elif tot_mag < self.MIN_SAFE_DIST:
                        self.drive.move(15, 15)
        return lidar_arr
