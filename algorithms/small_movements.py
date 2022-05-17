import interfaces
import logging
import core
import math
import algorithms.heading_hold as hh
import itertools
import time
import geomath
from core import constants


def backup(target_distance, speed = -200):
        """
        Backs rover up for a specified distance at specified speed

        Parameters:
        -----------
            target_distance (float) - distance to travel backwards (meters)
            speed (int16) - the speed to drive right and left motors (between -1000 and -1)
        """

        # Force distance to be positive and speed to be negative
        target_distance = abs(target_distance)
        speed = -abs(speed)

        # Initialize
        distance_traveled = 0
        start_latitude, start_longitude = interfaces.nav_board.location()

        # Check distance traveled until target distance is reached
        while(distance_traveled < target_distance):
            interfaces.drive_board.send_drive(speed, speed)
            current_latitude, current_longitude = interfaces.nav_board.location()
            bearing, distance_traveled = geomath.haversine(start_latitude, start_longitude, current_latitude, current_longitude)
            distance_traveled *= 1000 # convert km to m
            print(f"Backing Up: {distance_traveled} meters / {target_distance} meters")
            time.sleep(core.EVENT_LOOP_DELAY)

        # Stop rover
        print(f"Backing Up: COMPLETED")
        interfaces.drive_board.stop()


def time_drive(distance, direction):
    """
    Drives the rover in a straiht line for time 'goal_time'.
    'goal_time' is calculated by dividing goal distance by the constant METERS_PER_SECOND

    Parameters:
    -----------
        distance (float) - distance to travel
        direction (True/False) - True for forwards, False for backwards.
    """
    goal_time = distance / constants.METERS_PER_SECOND
    t1 = time.time()
    t2 = time.time()
    
    if direction == True:
        while t2 - t1 < goal_time:
            t2 = time.time()
            interfaces.drive_board.send_drive(constants.MAX_DRIVE_POWER, constants.MAX_DRIVE_POWER)
    if direction == False:
        while t2 - t1 < goal_time:
            t2 = time.time()
            interfaces.drive_board.send_drive(-constants.MAX_DRIVE_POWER, -constants.MAX_DRIVE_POWER)

    interfaces.drive_board.stop()
    
    def rotate_angle(angle):
        '''
        Rotates the Rover a specified angle.
        This is completely theoretical right now.
        I also wanna die -Donovan

        Perameters:
        -----------
            angle (degrees) - turn 'angle' degrees
        '''
        pass
    
    def dance_party():
        '''
        Spin in place multiple times.
        '''
        pass