import interfaces
from geopy import Point
from geopy.distance import VincentyDistance
from algorithms import geomath


def plan_avoidance_route(angle, distance, obstacle_lat, obstacle_lon, type="Rectangle"):
    """
    Plans a series of GPS coordinates around an obstacle, that can be used to navigate
    around it and avoid hazardous conditions for the rover

    Parameters:
    -----------
        angle - the angle of the obstacle
        distance - the distance from the rover to the obstacle
        obstalce_lat - The latitude of the given obstacle to avoid
        obstacle_lon - The longitude of the given obstacle to avoid
        type - the type of avoidance route, can be "Circle" or "Rectangle" (default)

    Returns:
    --------
        A list of points around the obstacle that provide a safe path of traversal

    """
    if type == "Rectangle":
        points = []
        # Find point 3.5m to the right of our current location
        right_2M_lat, right_2M_lon = coords_obstacle(
            3.5,
            interfaces.nav_board.location()[0],
            interfaces.nav_board.location()[1],
            (angle + 90) % 360,
        )
        points.append((right_2M_lat, right_2M_lon))

        # Now find point 5.5 times the original distance ahead of last point
        ahead_5_5X_lat, ahead_5_5X_lon = coords_obstacle(5.5 * distance, right_2M_lat, right_2M_lon, angle)
        points.append((ahead_5_5X_lat, ahead_5_5X_lon))

        # Now move left 3.5m
        left_2M_lat, left_2M_lon = coords_obstacle(3.5, ahead_5_5X_lat, ahead_5_5X_lon, (angle - 90) % 360)
        points.append((left_2M_lat, left_2M_lon))

        # return the GPS coordinates on our route
        return points

    elif type == "Circle":
        bearing, radius = geomath.haversine(
            interfaces.nav_board.location()[0], interfaces.nav_board.location()[1], obstacle_lat, obstacle_lon
        )
        # Convert to meters
        radius *= 1000
        # Add in an additional .5 meters to compensate for regular GPS inaccuracy
        radius += 0.5
        points = []

        increments = 4
        angle_increments = 90 / (increments - 1)
        point_angle = (angle - 90) % 360 + angle_increments

        for i in range(increments):
            points.append(coords_obstacle(radius, obstacle_lat, obstacle_lon, point_angle))
            point_angle += angle_increments

        # return the GPS coordinates on our route
        return points


def coords_obstacle(distMeters, lat1, lon1, bearing):
    """
    Calculates the lat and lon coords of the obstacle given the current position

    Parameters:
    -----------
        distMeters - the distance from the current position to the obstacle
        lat1 - the latitude of the current position
        lon1 - the longitude of the current position
        bearing - the angle from the current position to the obstacle

    Returns:
    --------
        The coords of the obstacle
    """
    destination = VincentyDistance(meters=distMeters).destination(Point(lat1, lon1), bearing)
    lat2, lon2 = destination.latitude, destination.longitude
    return (lat2, lon2)
