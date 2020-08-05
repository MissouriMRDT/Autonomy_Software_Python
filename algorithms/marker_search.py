import math
import core.constants as constants


# Performs a calculation for the next target gps on a spiral search pattern
# Start and current are a Coordinate lat/lon
def calculate_next_coordinate(start, former_goal):
    # these need to be mapped to radians for math stuff, otherwise they break in very terrible ways. We might want to just use Haversine here as well?
    diff_lat = (start.lat - former_goal.lat) * 1000
    diff_lon = (start.lon - former_goal.lon) * 1000

    r = math.sqrt(diff_lat ** 2 + diff_lon ** 2)
    theta = r / constants.SEARCH_DISTANCE

    # Add delta theta to calculate new point with
    theta += constants.DELTA_THETA # We should definitely decrease theta from math.pi/2 to pi/4, pi/6 or even pi/8
    # needs additional scaling at some point to handle large search patterns appropriately.
    r = constants.SEARCH_DISTANCE * theta

    diff_lat = r * math.sin(theta)
    diff_lon = r * math.cos(theta)

    return constants.Coordinate(start.lat + diff_lat/1000, start.lon + diff_lon/1000)
