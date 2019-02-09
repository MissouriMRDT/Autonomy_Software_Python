import math
import constants

# Performs a calculation for the next target gps on a spiral search pattern
# Start and current are a Coordinate lat/lon
def calculate_next_coordinate(start, current):

    diff_lat = start.lat - current.lat
    diff_lon = start.lon - current.lon

    r = math.sqrt(diff_lat ** 2 + diff_lon ** 2)
    theta = r / constants.SEARCH_DISTANCE

    # Add delta theta to calculate new point with
    theta += constants.DELTA_THETA

    r = constants.SEARCH_DISTANCE * constants.DELTA_THETA

    diff_lat = r * math.sin(theta)
    diff_lon = r * math.cos(theta)

    return constants.Coordinate(start.lat + diff_lat, start.lon + diff_lon)
