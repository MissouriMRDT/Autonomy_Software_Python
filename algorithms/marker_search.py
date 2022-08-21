# Find more info on archimedean spirals here https://www.britannica.com/science/spiral-mathematics

import math
import core


def calculate_next_coordinate(start, former_goal):
    """
    Performs a calculation for the next target gps on a spiral search pattern. Uses the
    archimedean spiral formula r (radius) = a (constant) θ (angle) to generate a series
    of GPS coordiantes in a spiral pattern, suitable for traversing terrain and sweeping
    the area with camera

    Parameters:
    -----------
        start (gps coordinate) - the coordinate at which Search Pattern was initated
        former goal (gps coordinate) - the previous gps coordinate in the search pattern

    Returns:
    --------
        gps coordinate - the next point to travers to in the spiral
    """

    # these need to be mapped to radians for math stuff, otherwise they break in very
    # terrible ways. We might want to just use Haversine here as well?
    diff_lat = (start.lat - former_goal.lat) * 1000
    diff_lon = (start.lon - former_goal.lon) * 1000

    # Calculate the radius of the current leg of the spiral
    r = math.sqrt(diff_lat ** 2 + diff_lon ** 2)

    # Formula for archimedes spiral is r = aθ, calculate current theta using a known a
    # (search distance) and a known r (distance to center, or starting point)
    theta = r / (core.SEARCH_DISTANCE / 1000)

    # Add delta theta to theta
    theta += core.DELTA_THETA

    # Now that we have a new θ, calculate the new radius given a and θ
    r = (core.SEARCH_DISTANCE / 1000) * theta

    # convert back to cartesian coordiantes
    diff_lat = r * math.sin(theta)
    diff_lon = r * math.cos(theta)

    # Add the offsets to the starting position to calculate next point in spiral
    return core.Coordinate(start.lat + diff_lat / 1000, start.lon + diff_lon / 1000)
