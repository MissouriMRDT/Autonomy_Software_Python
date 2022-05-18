import interfaces
import logging
import core
import math
import algorithms.heading_hold as hh
import algorithms
import itertools
import time
from core import constants
import asyncio

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
            bearing, distance_traveled = algorithms.geomath.haversine(start_latitude, start_longitude, current_latitude, current_longitude)
            distance_traveled *= 1000 # convert km to m
            print(f"Backing Up: {distance_traveled} meters / {target_distance} meters")
            time.sleep(core.EVENT_LOOP_DELAY)

        # Stop rover
        print(f"Backing Up: COMPLETED")
        interfaces.drive_board.stop()


def time_drive(distance, is_forward):
    """
    Drives the rover in a straiht line for time 'goal_time'.
    'goal_time' is calculated by dividing goal distance by the constant METERS_PER_SECOND

    Parameters:
    -----------
        distance (float) - distance to travel
        is_forward (True/False) - True for forwards, False for backwards.
    """
    goal_time = distance / constants.METERS_PER_SECOND
    t1 = time.time()
    t2 = time.time()
    
    if is_forward == True:
        while t2 - t1 < goal_time:
            t2 = time.time()
            interfaces.drive_board.send_drive(constants.MAX_DRIVE_POWER, constants.MAX_DRIVE_POWER)
            
    elif is_forward == False:
        while t2 - t1 < goal_time:
            t2 = time.time()
            interfaces.drive_board.send_drive(-constants.MAX_DRIVE_POWER, -constants.MAX_DRIVE_POWER)
            
    """
    I need to figure out how to do this asynchronously.
    This isn't necessarily an issue yet,
    but I imagine this function will be used in situations where vision is needed.
    And that doesn't work right now.
    """

    interfaces.drive_board.stop()
    
def rotate_rover(angle):
    '''
    Rotates the Rover a specified angle.
    This is completely theoretical right now.
    I also wanna die -Donovan

    Perameters:
    -----------
        angle (degrees) - turn 'angle' degrees. Negative turns left, positive turns right.
    '''
    
    start_heading = interfaces.nav_board.heading()
    target_heading = (start_heading + angle) % 360
    going = True
    while going:
        if (interfaces.nav_board.heading() + 5) % 360 < target_heading:
            interfaces.drive_board.send_drive(constants.MIN_DRIVE_POWER, -constants.MAX_DRIVE_POWER)
            going = True
        elif (interfaces.nav_board.heading() - 5) % 360 > target_heading:
            interfaces.drive_board.send_drive(-constants.MAX_DRIVE_POWER, constants.MIN_DRIVE_POWER)
            going = True
        else:
            going = False

def dance_party():
    '''
    Spin in place multiple times.
    '''
    pass