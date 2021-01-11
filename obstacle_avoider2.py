import algorithms
import core
import logging
import interfaces
from interfaces import drive_board, nav_board
from geopy import Point
from geopy.distance import distance, VincentyDistance
import algorithms.geomath as geomath
import matplotlib.pyplot as plt
import gmplot
import algorithms.heading_hold

# Create the map plotter:
apikey = ""  # (your API key here)
gmap = gmplot.GoogleMapPlotter(37.95139193343744, -91.76901326338806, 14, apikey=apikey)


def get_relative_angle_subtract(angle, angle2):
    diff = angle - angle2
    if diff > 0:
        return diff
    else:
        return 360 + diff


def plan_avoidance_route(angle, obstacle_lat, obstacle_lon):
    bearing, radius = geomath.haversine(nav_board.location()[0], nav_board.location()[1], obstacle_lat, obstacle_lon)
    radius *= 1000
    points = []

    increments = 4
    angle_increments = 180 / increments
    point_angle = get_relative_angle_subtract(angle, 180) + angle_increments

    for i in range(increments):
        print(i)
        points.append(coords_obstacle(radius, obstacle_lat, obstacle_lon, point_angle))
        point_angle += angle_increments
    print(point_angle)

    # return the GPS coordinates on our route
    return points


def main() -> None:
    """
    Main function for example script, tests geomath code
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    points = []

    # Finding the obstacle
    angle, distance = obstacle_detection()

    # Saving our starting location
    one_meter_from_obstacle = nav_board.location()

    # find the gps coordinate of the obstacle
    obstacle_lat, obstacle_lon = coords_obstacle(distance, nav_board.location()[0], nav_board.location()[1], angle)

    points = plan_avoidance_route(angle, obstacle_lat, obstacle_lon)
    # points.insert(0, (nav_board.location()[0], nav_board.location()[1]))

    """
    # Outline the Golden Gate Park:
    golden_gate_park = zip(
        *[
            (interfaces.nav_board.location()[0], interfaces.nav_board.location()[1]),
            (right_2M_lat, right_2M_lon),
            (ahead_4X_lat, ahead_4X_lon),
            (left_2M_lat, left_2M_lon),
        ]
    )
    gmap.polygon(*golden_gate_park, color="cornflowerblue", edge_width=10)
    gmap.draw("map.html")
    """

    previous_loc = one_meter_from_obstacle

    for point in points:
        new_lat, new_lon = point
        while (
            algorithms.gps_navigate.get_approach_status(
                core.constants.Coordinate(new_lat, new_lon), nav_board.location(), previous_loc
            )
            == core.constants.ApproachState.APPROACHING
        ):
            logger.info(f"Target coordinates: Lat: {new_lat}, Lon: {new_lon}")
            left, right = algorithms.gps_navigate.calculate_move(
                core.constants.Coordinate(new_lat, new_lon), nav_board.location(), previous_loc, 250
            )
            logger.debug(f"Navigating: Driving at ({left}, {right})")
            core.rovecomm.write(drive_board.send_drive(left, right))

        core.rovecomm.write(drive_board.send_drive(0, 0))
        previous_loc = core.constants.Coordinate(new_lat, new_lon)


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
