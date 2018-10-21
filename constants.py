from enum import Enum

# Navigation Parameters
WIDTH = 640.0  # pixels
FIELD_OF_VIEW = 40.0  # degrees
TARGET_DISTANCE = 0.4  # meters
RADIUS = .063  # meters
SCALING_FACTOR = 10.0  # pixel-meters
DRIVE_POWER = 25  # percent

# Range at which we switch from GPS to optical tracking
VISION_RANGE = 0.007  # kilometers

# RoveComm Autonomy Control DataIDs
class DataID(Enum):
    ENABLE_AUTONOMY = 2576
    DISABLE_AUTONOMY = 2577
    ADD_WAYPOINT = 2578
    CLEAR_WAYPOINTS = 2579
    WAYPOINT_REACHED = 2580
