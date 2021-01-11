from enum import IntEnum, Enum
import collections
import math
import json

# Autonomy General Configuration
EVENT_LOOP_DELAY = 0.1  # seconds

# Navigation Parameters
WIDTH = 640.0  # pixels
FIELD_OF_VIEW = 40.0  # degrees
TARGET_DISTANCE = 0.4  # meters
RADIUS = 0.063  # meters
SCALING_FACTOR = 10.0  # pixel-meters
DRIVE_POWER = 250  # -1000 to 1000, normally 250 dropped lower for early testing to be safe
WAYPOINT_DISTANCE_THRESHOLD = 1.5  # maximum threshold in meters between rover and waypoint
BEARING_FLIP_THRESHOLD = 30.0  # 180 +/- this many degrees counts as a flip in bearing

SEARCH_DISTANCE = 0.008
DELTA_THETA = math.pi / 2

# LiDAR maximum distance before we decide we would yeet off a cliff.
LIDAR_MAXIMUM = 250  # 2.5m to test early, need to determine actual value.

# Range at which we switch from GPS to optical tracking
VISION_RANGE = 0.007  # kilometers

Coordinate = collections.namedtuple("Coordinate", ["lat", "lon"])

# RoveComm Numerical Values
rovecomm_event_list = open("core/rovecomm_values.json", "r").read()
rovecomm_event_list = json.loads(rovecomm_event_list)


# RoveComm Autonomy Control DataIDs
class DataID(IntEnum):
    ENABLE_AUTONOMY = 11100
    DISABLE_AUTONOMY = 11101
    ADD_WAYPOINT = 11102
    CLEAR_WAYPOINTS = 11103
    WAYPOINT_REACHED = 11104


class ApproachState(Enum):
    APPROACHING = 1
    CLOSE_ENOUGH = 2
    PAST_GOAL = 3


class GPSData:
    def __int__(self, goal, start):
        self.goal = goal
        self.start = start

    def data(self):
        return self.goal, self.start
