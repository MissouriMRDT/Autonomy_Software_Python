import interfaces
from geopy import Point
from geopy.distance import VincentyDistance
from algorithms import geomath


def get_relative_angle_subtract(angle, angle2):
    """
    Takes two angles and subtracts them, returns an angle no greater than 360, and no less
    than 0

    Parameters:
        angle1 - the angle of the first object (say an obstacle)
        angle2 - the angle of the second object (say the rover)

    Returns:
        relative angle - The angles subtract, but kept within 360
    """
    diff = angle - angle2
    if diff > 0:
        return diff
    else:
        return 360 + diff


def get_relative_angle_add(angle, angle2):
    """
    Takes in two angles and adds them together

    Parameters:
        angle - the angle of the first object (say an obstacle)
        angle2 - the angle of the second object (say the rover)

    Returns:
        The sum of the two angles no greater than 360

    """
    diff = angle + angle2
    if diff <= 360:
        return diff
    else:
        return diff - 360


def plan_avoidance_route(angle, distance, obstacle_lat, obstacle_lon, type="Rectangle"):
    """
    Plans a series of GPS coordinates around an obstacle, that can be used to navigate
    around it and avoid hazardous conditions for the rover

    Parameters:
        angle - the angle of the obstacle
        distance - the distance from the rover to the obstacle
        type - the type of avoidance route, can be "Circle" or "Rectangle" (default)

    Returns:
        A list of points around the obstacle

    """
    if type == "Rectangle":
        points = []
        # Find point 2m to the right of our current location
        right_2M_lat, right_2M_lon = coords_obstacle(
            3.5,
            interfaces.nav_board.location()[0],
            interfaces.nav_board.location()[1],
            get_relative_angle_add(angle, 90),
        )
        points.append((right_2M_lat, right_2M_lon))

        # Now find point 4m ahead of last point
        ahead_4X_lat, ahead_4X_lon = coords_obstacle(5.5 * distance, right_2M_lat, right_2M_lon, angle)
        points.append((ahead_4X_lat, ahead_4X_lon))

        # Now move left 2m
        left_2M_lat, left_2M_lon = coords_obstacle(
            3.5, ahead_4X_lat, ahead_4X_lon, get_relative_angle_subtract(angle, 90)
        )
        points.append((left_2M_lat, left_2M_lon))

        # return the GPS coordinates on our route
        return points

    elif type == "Circle":
        bearing, radius = geomath.haversine(
            interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], obstacle_lat, obstacle_lon
        )
        radius *= 1000
        radius += 2.0
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


def coords_obstacle(distMeters, lat1, lon1, bearing):
    """
    Calculates the lat and lon coords of the obstacle given the current position

    Parameters:
        distMeters - the distance from the current position to the obstacle
        lat1 - the latitude of the current position
        lon1 - the longitude of the current position
        bearing - the angle from the current position to the obstacle

    Returns:
        The coords of the obstacle
    """
    # given: lat1, lon1, bearing, distMiles
    destination = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)
    lat2, lon2 = destination.latitude, destination.longitude
    return (lat2, lon2)
