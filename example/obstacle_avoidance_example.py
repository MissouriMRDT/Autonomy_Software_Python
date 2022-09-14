import time
import algorithms
import core
import core.constants
import logging
import interfaces
from geopy import Point
from geopy.distance import VincentyDistance
import algorithms.heading_hold
import algorithms.obstacle_avoider


def main() -> None:
    """
    Test script for obstacle avoidance (not detection)
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    points = []
    # Finding the obstacle
    angle, distance = obstacle_detection()

    # Saving our starting location
    one_meter_from_obstacle = interfaces.nav_board.location()

    # find the gps coordinate of the obstacle
    obstacle_lat, obstacle_lon = algorithms.obstacle_avoider.coords_obstacle(
        distance, interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], angle
    )

    # Chart the course around the obstacle
    points = algorithms.obstacle_avoider.plan_avoidance_route(angle, distance, obstacle_lat, obstacle_lon)

    time.sleep(1)

    previous_loc = one_meter_from_obstacle

    # Drives to each of the points in the list of points around the object in sequence
    for point in points:
        new_lat, new_lon = point
        logger.info(f"Driving towards : Lat: {new_lat}, Lon: {new_lon} now")
        while (
            algorithms.gps_navigate.get_approach_status(
                core.Coordinate(new_lat, new_lon), interfaces.nav_board.location(), previous_loc
            )
            == core.ApproachState.APPROACHING
        ):
            left, right = algorithms.gps_navigate.calculate_move(
                core.Coordinate(new_lat, new_lon), interfaces.nav_board.location(), previous_loc, core.MAX_DRIVE_POWER
            )
            logger.debug(f"Navigating: Driving at ({left}, {right})")
            interfaces.drive_board.send_drive(left, right)
            time.sleep(0.1)

        interfaces.drive_board.stop()
        previous_loc = core.Coordinate(new_lat, new_lon)


def obstacle_detection():
    # returns angle, distance
    return (interfaces.nav_board.heading(), 1)


if __name__ == "__main__":
    # Run main()
    main()
