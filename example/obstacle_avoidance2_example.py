from core.constants import VISION_RANGE
import algorithms
import core
import logging
import interfaces
from interfaces import drive_board, nav_board
from geopy import Point
from geopy.distance import VincentyDistance
import algorithms.geomath as geomath
import matplotlib.pyplot as plt
import gmplot
import algorithms.heading_hold
import time, cv2
from algorithms import obstacle_avoider


# Create the map plotter:
apikey = ""  # (your API key here)
gmap = gmplot.GoogleMapPlotter(37.95139193343744, -91.76901326338806, 14, apikey=apikey)
logger = logging.getLogger(__name__)
logger.info("Executing function: main()")


def simulate_obstacle_avoidance(DETECT_OBSTACLE=True):
    points = []

    # Finding the obstacle
    angle, distance = obstacle_detection()

    # Saving our starting location
    one_meter_from_obstacle = nav_board.location()

    # find the gps coordinate of the obstacle
    obstacle_lat, obstacle_lon = obstacle_avoider.coords_obstacle(
        distance, nav_board.location()[0], nav_board.location()[1], angle
    )

    points = obstacle_avoider.plan_avoidance_route(angle, distance, obstacle_lat, obstacle_lon, type="Circle")

    previous_loc = one_meter_from_obstacle
    FOUND_OBSTACLE = False

    # Drives to each of the points in the list of points around the object in sequence
    for point in points:
        new_lat, new_lon = point
        logger.info(f"Driving towards : Lat: {new_lat}, Lon: {new_lon} now")
        while (
            algorithms.gps_navigate.get_approach_status(
                core.constants.Coordinate(new_lat, new_lon), interfaces.nav_board.location(), previous_loc, 0.5
            )
            == core.constants.ApproachState.APPROACHING
        ):
            # logger.info(f"Target coordinates: Lat: {new_lat}, Lon: {new_lon}")
            reg_img = core.vision.camera_handler.grab_regular()
            left, right = algorithms.gps_navigate.calculate_move(
                core.constants.Coordinate(new_lat, new_lon), interfaces.nav_board.location(), previous_loc, 250
            )
            if not FOUND_OBSTACLE and DETECT_OBSTACLE:
                tag_cascade = cv2.CascadeClassifier("cascade.xml")

                gray = cv2.cvtColor(reg_img, cv2.COLOR_BGR2GRAY)

                tags = tag_cascade.detectMultiScale(gray, 1.3, 5)

                if len(tags) > 0:
                    logger.info("Found an AR Tag!")
                    FOUND_OBSTACLE = True

                for (x, y, w, h) in tags:
                    reg_img = cv2.rectangle(reg_img, (x, y), (x + w, y + h), (255, 0, 0), 2)

            if FOUND_OBSTACLE:
                break

            logger.debug(f"Navigating: Driving at ({left}, {right})")
            interfaces.drive_board.send_drive(left, right)
            cv2.imshow("img", reg_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.1)

        if FOUND_OBSTACLE:
            break
        interfaces.drive_board.stop()
        previous_loc = core.constants.Coordinate(new_lat, new_lon)


def main() -> None:
    """
    Main function for example script, tests geomath code
    """
    simulate_obstacle_avoidance()
    time.sleep(2)
    simulate_obstacle_avoidance(False)


def obstacle_detection():
    # returns angle, distance
    return (nav_board.heading(), 1)


if __name__ == "__main__":
    # Run main()
    main()
