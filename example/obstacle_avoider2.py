from core.constants import VISION_RANGE
import algorithms
import core
import logging
import interfaces
from interfaces import drive_board, nav_board
from geopy import Point
from geopy.distance import distance, VincentyDistance
import algorithms.geomath as geomath
import matplotlib.pyplot as plt
#import gmplot
import algorithms.heading_hold
import time, cv2


# Create the map plotter:
#apikey = ""  # (your API key here)
#gmap = gmplot.GoogleMapPlotter(37.95139193343744, -91.76901326338806, 14, apikey=apikey)
logger = logging.getLogger(__name__)
logger.info("Executing function: main()")


def get_relative_angle_subtract(angle, angle2):
    diff = angle - angle2
    if diff > 0:
        return diff
    else:
        return 360 + diff


def plan_avoidance_route(angle, obstacle_lat, obstacle_lon):
    bearing, radius = geomath.haversine(nav_board.location()[0], nav_board.location()[1], obstacle_lat, obstacle_lon)
    radius *= 1000
    #radius += 2.0
    points = []

    increments = 4
    angle_increments = 90 / (increments - 1)
    point_angle = get_relative_angle_subtract(angle, 90)

    for i in range(increments):
        print(point_angle)
        points.append(coords_obstacle(radius, obstacle_lat, obstacle_lon, point_angle))
        point_angle += angle_increments

    # return the GPS coordinates on our route
    return points


def simulate_obstacle_avoidance(DETECT_OBSTACLE=True):
    points = []
    obstacle = []
    print(nav_board.heading())
    # Finding the obstacle
    while obstacle == []:
        depth_data = core.vision.camera_handler.grab_depth_data()
        obstacle = algorithms.obstacle_detector.detect_obstacle(depth_data, 1, 3)
        reg_img = core.vision.camera_handler.grab_regular()
        core.vision.camera_handler.feed_handler.handle_frame("regular", reg_img)

    reg_img = core.vision.camera_handler.grab_regular()
    reg_img = cv2.resize(reg_img, (int(1280 / 2), int(720 / 2)))

    angle, distance, _ = algorithms.obstacle_detector.track_obstacle(
            depth_data, obstacle, True, reg_img
        )
    angle = (nav_board.heading() + angle) % 360
    core.vision.camera_handler.feed_handler.handle_frame("regular", reg_img)

    angle, distance = obstacle_detection()

    # Saving our starting location
    one_meter_from_obstacle = nav_board.location()

    # find the gps coordinate of the obstacle
    obstacle_lat, obstacle_lon = coords_obstacle(distance, nav_board.location()[0], nav_board.location()[1], angle)

    points = plan_avoidance_route(angle, obstacle_lat, obstacle_lon)

    previous_loc = one_meter_from_obstacle

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
        
            logger.debug(f"Navigating: Driving at ({left}, {right})")
            interfaces.drive_board.send_drive(left, right)
            #cv2.imshow("img", reg_img)
            #if cv2.waitKey(1) & 0xFF == ord("q"):
            #    break
            time.sleep(0.1)

        #if FOUND_OBSTACLE:
        #    break
        interfaces.drive_board.stop()
        previous_loc = core.constants.Coordinate(new_lat, new_lon)


def main() -> None:
    """
    Main function for example script, tests geomath code
    """
    simulate_obstacle_avoidance()
    time.sleep(2)
    #simulate_obstacle_avoidance(False)


def coords_obstacle(distMeters, lat1, lon1, bearing):
    # given: lat1, lon1, bearing, distMiles
    destination = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)
    lat2, lon2 = destination.latitude, destination.longitude
    return (lat2, lon2)


def obstacle_detection():
    # returns angle, distance
    return (nav_board.heading(), 1)


if __name__ == "__main__":
    # Run main()
    main()
