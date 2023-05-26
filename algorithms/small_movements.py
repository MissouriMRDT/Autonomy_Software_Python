import interfaces
import logging
import core
import algorithms
import time


def backup(start_latitude, start_longitude, target_distance, speed=-200):
    """
    Backs rover up for a specified distance at specified speed

    :param start_lat: The start reference to calculate distance from.
    :param start_long: The start reference to calculate distance from.
    :param target_distance: Distance to travel backwards (meters)
    :param speed: The speed to drive right and left motors (between -1000 and -1)

    :returns done: Whether or not the rover has backed up the target distance.
    """
    # Setup logger for function.
    logger = logging.getLogger(__name__)

    # Force distance to be positive and speed to be negative
    target_distance = abs(target_distance)
    speed = -abs(speed)

    # Get total distance traveled so far.
    current_latitude, current_longitude = interfaces.nav_board.location(force_absolute=True)
    bearing, distance_traveled = algorithms.geomath.haversine(
        start_latitude, start_longitude, current_latitude, current_longitude
    )
    # Convert to km to m.
    distance_traveled *= 1000

    # Check distance traveled until target distance is reached
    if distance_traveled < target_distance:
        # Send drive command.
        interfaces.drive_board.send_drive(speed, speed)
        # Print log.
        logger.info(f"Backing Up: {distance_traveled} meters / {target_distance} meters")
        # Return false since we are still backing up.
        return False
    else:
        # Stop rover drive.
        interfaces.drive_board.stop()
        # Print log.
        logger.info(f"Backing Up: COMPLETED")
        # Return true since we have reversed specified distance.
        return True


def time_drive(distance):
    """
    Drives the rover in a straight line for time 'goal_time'.
    'goal_time' is calculated by dividing goal distance by the constant METERS_PER_SECOND

    Parameters:
    -----------
        distance (float) - distance to travel. Negative for reverse.
    """

    goal_time = abs(distance) / core.METERS_PER_SECOND
    t1 = time.time()
    t2 = time.time()

    if distance > 0:
        while t2 - t1 < goal_time:
            t2 = time.time()
            interfaces.drive_board.send_drive(core.MAX_DRIVE_POWER, core.MAX_DRIVE_POWER)
            time.sleep(core.EVENT_LOOP_DELAY)

    elif distance < 0:
        while t2 - t1 < goal_time:
            t2 = time.time()
            interfaces.drive_board.send_drive(-core.MAX_DRIVE_POWER, -core.MAX_DRIVE_POWER)
            time.sleep(core.EVENT_LOOP_DELAY)

    """
    I need to figure out how to do this asynchronously.
    This isn't necessarily an issue yet,
    but I imagine this function will be used in situations where vision is needed.
    And that doesn't work right now.
    """

    interfaces.drive_board.stop()


def dance_party():
    """
    Spin in place multiple times.
    """
    pass
