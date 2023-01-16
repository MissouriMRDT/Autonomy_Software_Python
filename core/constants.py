#
# Mars Rover Design Team
# constants.py
#
# Created on Jul 08, 2020
# Updated on Aug 21, 2022
#

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
WAYPOINT_DISTANCE_THRESHOLD = 1.5  # maximum threshold in meters between rover and waypoint
BEARING_FLIP_THRESHOLD = 30.0  # 180 +/- this many degrees counts as a flip in bearing
METERS_PER_SECOND = 0.762  # at speeds (450, 450) **CHANGE FOR UTAH TERRAIN
AR_SKEW_THRESHOLD = 30  # min angle allowed between tags for approaching gate to skip first leg
MAX_DRIVE_POWER = 300  # -1000 to 1000, normally 250 dropped lower for early testing to be safe
MIN_DRIVE_POWER = 50

INCREMENT = 1.25  #Rate at which speed in search pattern increases
DECREMENT = 1.25  #Rate at which speed in search pattern increases
SP_RIGHT_MAX = 180
SP_RIGHT_MIN = 90
SP_LEFT_MAX = 300
SP_LEFT_MIN = 240
LOWER_RATE_OF_CHANGE = 0.0123

# Search Pattern Parameters
SEARCH_DISTANCE = 20  # meters
DELTA_THETA = math.pi / 4

# Vision Parameters
MAX_DETECTION_ATTEMPTS = 15  # This should be about 1 second

# LiDAR maximum distance before we decide we would yeet off a cliff.
LIDAR_MAXIMUM = 250  # 2.5m to test early, need to determine actual value.

# Range at which we switch from GPS to optical tracking
VISION_RANGE = 0.007  # meters
MIN_OBSTACLE_PIXEL_AREA = 400  # minimum contour area in pixels of detected obstacle
DEPTH_STEP_SIZE = 0.5  # Depth in meters that each segment of obstacle detection will run on
NUM_DEPTH_SEGMENTS = 3  # Number of segments of depth map to actually run contour detection on
ZED_Z_OFFSET = 0.34  # Offset in meters.

Coordinate = collections.namedtuple("Coordinate", ["lat", "lon"])

# RoveComm Numerical Values used to map events to enumerations to send over rovecomm
rovecomm_event_list = open("core/rovecomm_values.json", "r").read()
rovecomm_event_list = json.loads(rovecomm_event_list)

# RoveComm manifest
manifest = {}

# Outgoing communication ports
UDP_OUTGOING_PORT = None
TCP_OUTGOING_PORT = 11111

# ArUco Detection Occurrences
FRAMES_DETECTED = 5


class ApproachState(Enum):
    APPROACHING = 1
    CLOSE_ENOUGH = 2
    PAST_GOAL = 3


class OperationState(IntEnum):
    TELEOP = 0
    AUTONOMY = 1
    REACHED_MARKER = 2
