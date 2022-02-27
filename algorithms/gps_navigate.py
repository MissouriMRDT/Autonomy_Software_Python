import logging
import math
import interfaces
import algorithms.geomath as geomath
import algorithms.heading_hold as hh
import core


def get_approach_status(goal, location, start, tolerance=core.WAYPOINT_DISTANCE_THRESHOLD):
    """
    Calculates the current approach status of the rover in reference to goal and starting location

    Returns:
    -------
        approach_status : enum
            whether we are APPROACHING, PAST_GOAL or CLOSE_ENOUGH
    """
    logger = logging.getLogger(__name__)

    (s_bearing, s_distance) = geomath.haversine(start.lat, start.lon, goal.lat, goal.lon)
    (c_bearing, c_distance) = geomath.haversine(location.lat, location.lon, goal.lat, goal.lon)

    distanceMeters = c_distance * 1000.0
    close_enough = distanceMeters < tolerance

    bearing_diff = (s_bearing - c_bearing) % 360.0
    past_goal = 180 - core.BEARING_FLIP_THRESHOLD <= bearing_diff <= 180 + core.BEARING_FLIP_THRESHOLD

    if past_goal:
        logger.info("PAST GOAL")
        return core.ApproachState.PAST_GOAL

    if close_enough:
        logger.info("CLOSE ENOUGH")
        return core.ApproachState.CLOSE_ENOUGH

    return core.ApproachState.APPROACHING


def calculate_move(goal, location, start, speed=150):
    """
    Calculates the necessary left and right speeds to keep the rover on course for goal location

    Returns:
    -------
        left_speed : int16
            the desired left speed between -1000, 1000
        right speed: int16
            the desired right speed between -1000, 1000
    """
    logger = logging.getLogger(__name__)

    (target_heading, target_distance) = geomath.haversine(location.lat, location.lon, goal.lat, goal.lon)

    logger.debug(f"Target distance: {target_distance}")

    if target_distance < 0.01:
        speed = 100

    goal_heading = target_heading
    logger.debug(f"Current heading: {interfaces.nav_board.heading()}, Goal: {goal_heading}")

    return hh.get_motor_power_from_heading(speed, goal_heading)
