from enum import IntEnum
import collections
import math

# Navigation Parameters
WIDTH = 640.0  # pixels
FIELD_OF_VIEW = 40.0  # degrees
TARGET_DISTANCE = 0.4  # meters
RADIUS = .063  # meters
SCALING_FACTOR = 10.0  # pixel-meters
DRIVE_POWER = 200  # -1000 to 1000

SEARCH_DISTANCE = 0.02
DELTA_THETA = math.pi / 2

# Range at which we switch from GPS to optical tracking
VISION_RANGE = 0.007  # kilometers

Coordinate = collections.namedtuple('Coordinate', ['lat', 'lon'])


# RoveComm Autonomy Control DataIDs
class DataID(IntEnum):
    ENABLE_AUTONOMY = 2576
    DISABLE_AUTONOMY = 2577
    ADD_WAYPOINT = 2578
    CLEAR_WAYPOINTS = 2579
    WAYPOINT_REACHED = 2580
