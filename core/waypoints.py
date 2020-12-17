import logging
from core import constants
from collections import deque
from interfaces import nav_board
from algorithms.gps_navigate import GPSData

# Module level variables
waypoints: deque = deque()
logger = logging.getLogger(__name__)
gps_data = None


def add_waypoint(packet_contents):
    longitude, latitude = packet_contents.data
    waypoint = constants.Coordinate(latitude, longitude)
    waypoints.append(waypoint)
    logger.info(f"Added Waypoint: lat ({latitude}), lon({longitude})")


def clear_waypoints(packet_contents):
    waypoints.clear()
    logger.info("Cleared all waypoints")


def get_waypoint() -> GPSData:
    # Pop off a waypoint from the queue if there is currently none
    if gps_data is None:
        return set_gps_waypoint()

    return gps_data


def set_goal(goal):
    global gps_data

    # Set the goal to the passed in goal
    gps_data.goal = goal


def set_gps_waypoint():
    global gps_data
    gps_data = GPSData()
    gps_data.start = nav_board.location()

    current_goal = waypoints.popleft()
    gps_data.goal = current_goal

    logger.info(f"Set Waypoint Target: lat ({current_goal[0]}), lon({current_goal[1]})")
    return gps_data
