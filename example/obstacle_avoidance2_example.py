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

# import gmplot
import algorithms.heading_hold
import time, cv2
from algorithms import obstacle_avoider


# Create the map plotter:
# apikey = ""  # (your API key here)
# gmap = gmplot.GoogleMapPlotter(37.95139193343744, -91.76901326338806, 14, apikey=apikey)
logger = logging.getLogger(__name__)
logger.info("Executing function: main()")


def simulate_obstacle_avoidance(DETECT_OBSTACLE=True):
    points = []
    obstacle = []
    previous_loc = interfaces.nav_board.location()
    target = core.Coordinate(37.95143624790924, -91.76908311165961)

    cv2.namedWindow("img")
    print(target)
    while (
        algorithms.gps_navigate.get_approach_status(
            target,
            interfaces.nav_board.location(),
            previous_loc,
            0.5,
        )
        == core.constants.ApproachState.APPROACHING
    ):
        left, right = algorithms.gps_navigate.calculate_move(
            target,
            interfaces.nav_board.location(),
            previous_loc,
            250,
        )
        reg_img = core.vision.camera_handler.grab_regular()

        reg_img = cv2.resize(reg_img, (1280, 720))
        cv2.imshow("img", reg_img)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
        logger.debug(f"Navigating: Driving at ({left}, {right})")
        interfaces.drive_board.send_drive(left, right)
        time.sleep(0.01)

    reg_img = core.vision.camera_handler.grab_regular()
    depth_matrix = core.vision.camera_handler.grab_depth_data()

    mask, lower = algorithms.obstacle_detector.get_floor_mask(
        reg_img, int(reg_img.shape[1] / 2), int(reg_img.shape[0] / 2)
    )

    depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)

    obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 0.1, 5)

    reg_img = cv2.resize(reg_img, (int(1280 / 2), int(720 / 2)))

    if obstacle != []:
        angle, distance, _ = algorithms.obstacle_detector.track_obstacle(depth_matrix, obstacle, True, reg_img)

    reg_img = cv2.resize(reg_img, (1280, 720))
    cv2.imshow("img", reg_img)

    angle = (nav_board.heading() + angle) % 360

    angle, distance = obstacle_detection()

    # Saving our starting location
    one_meter_from_obstacle = nav_board.location()

    # find the gps coordinate of the obstacle
    obstacle_lat, obstacle_lon = obstacle_avoider.coords_obstacle(
        distance, nav_board.location()[0], nav_board.location()[1], angle
    )

    points = obstacle_avoider.plan_avoidance_route(angle, distance, obstacle_lat, obstacle_lon, type="Circle")

    previous_loc = one_meter_from_obstacle

    # Drives to each of the points in the list of points around the object in sequence
    for point in points:
        new_lat, new_lon = point
        logger.info(f"Driving towards : Lat: {new_lat}, Lon: {new_lon} now")
        while (
            algorithms.gps_navigate.get_approach_status(
                core.constants.Coordinate(new_lat, new_lon),
                interfaces.nav_board.location(),
                previous_loc,
                0.5,
            )
            == core.constants.ApproachState.APPROACHING
        ):
            left, right = algorithms.gps_navigate.calculate_move(
                core.constants.Coordinate(new_lat, new_lon),
                interfaces.nav_board.location(),
                previous_loc,
                250,
            )

            logger.debug(f"Navigating: Driving at ({left}, {right})")
            interfaces.drive_board.send_drive(left, right)

            # Image
            reg_img = core.vision.camera_handler.grab_regular()
            depth_matrix = core.vision.camera_handler.grab_depth_data()

            #
            mask, lower = algorithms.obstacle_detector.get_floor_mask(
                reg_img, int(reg_img.shape[1] / 2), int(reg_img.shape[0] / 2)
            )

            depth_matrix = cv2.bitwise_and(depth_matrix, depth_matrix, mask=mask)

            obstacle = algorithms.obstacle_detector.detect_obstacle(depth_matrix, 0.1, 5)
            reg_img = cv2.resize(reg_img, (int(1280 / 2), int(720 / 2)))

            if obstacle != []:
                angle, distance, _ = algorithms.obstacle_detector.track_obstacle(depth_matrix, obstacle, True, reg_img)

            reg_img = cv2.resize(reg_img, (1280, 720))
            cv2.imshow("img", reg_img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            time.sleep(0.01)

        interfaces.drive_board.stop()
        previous_loc = core.constants.Coordinate(new_lat, new_lon)


def main() -> None:
    """
    Main function for example script, tests geomath code
    """
    simulate_obstacle_avoidance()
    time.sleep(2)
    # simulate_obstacle_avoidance(False)


def obstacle_detection():
    # returns angle, distance
    return (nav_board.heading(), 1)


if __name__ == "__main__":
    # Run main()
    main()
