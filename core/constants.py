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
IDLE_TIME_GPS_REALIGN = 5 # Second to sit in idle before realigning gps with relative.
IDLE_GPS_ACCUR_THRESH = 0.8 # The minimum meter accuracy needed to update rover position.

# Navigation Parameters
WIDTH = 640.0  # pixels
FIELD_OF_VIEW = 40.0  # degrees
TARGET_DISTANCE = 0.4  # meters
RADIUS = 0.063  # meters
SCALING_FACTOR = 10.0  # pixel-meters
WAYPOINT_DISTANCE_THRESHOLD = 2.0  # maximum threshold in meters between rover and waypoint
BEARING_FLIP_THRESHOLD = 30.0  # 180 +/- this many degrees counts as a flip in bearing
MAX_DRIVE_POWER = 600  # -1000 to 1000, normally 250 dropped lower for early testing to be safe
MIN_DRIVE_POWER = -250  # -1000 to 1000, normally 50
GATE_POINT_DISTANCES = 3.0
NAVIGATION_PATH_EXPIRATION_SECONDS = 10  # The time in seconds before a new path is force generated.
NAVIGATION_PATH_ROUTE_LENGTH = 30  # The length in meters that ASTAR will generate at one time.
METERS_PER_SECOND = 0.762  # at speeds (450, 450) **CHANGE FOR UTAH TERRAIN
AR_SKEW_THRESHOLD = 30  # min angle allowed between tags for approaching gate to skip first leg

# Approaching Gate Parameters.
GATE_WAYPOINT_THRESH = 0.3  # The minimum distance from end waypoint before we consider ourselves there.
GATE_NEAR_MARKER_THRESH = 0.4  # The closest the rover can get to a post.
GATE_UPDATE_PATH_MAX_MARKER_DISTANCE = (
    3  # The max distance we must be from the gate markers before we think tag detections will be accurate.
)
GATE_DRIVE_THROUGH_TIME = 10  # The amount of time to continue driving after going through gate.
RECENTER_GATE_THRESHOLD = 20

# Search Pattern Parameters
SEARCH_DISTANCE = 5  # meters
SEARCH_PATTERN_MAX_ERROR_FROM_PATH = 5  # The max distance the rover diverge off path before regen.
DELTA_THETA = math.pi / 4

# Obstacle Detection Parameters.
DETECTION_MODEL_CONF = 0.4
DETECTION_MODEL_IOU = 0.65
# Obstacle Avoidance Parameters.
AVOIDANCE_ENABLE_DISTANCE_THRESHOLD = 3.0  # Minimum distance rover must be from the waypoint before avoidance kicks in.
AVOIDANCE_OBJECT_DISTANCE_MIN = 2.0  # Closest rover can get to an obstacle.
AVOIDANCE_OBJECT_DISTANCE_MAX = 10.0  # Minimum distance rover must be from an obstacle before avoidance kicks in.
AVOIDANCE_OBJECT_ANGLE = 40  # The FOV of detection for obstacles.
AVOIDANCE_PATH_NODE_INCREMENT = 0.1  # The distance between each node. Path resolution in meters.
AVOIDANCE_PATH_EXPIRATION_SECONDS = 5  # The time in seconds before a new path is force generated.
AVOIDANCE_PATH_ROUTE_LENGTH = 40  # The length in meters that ASTAR will generate at one time.
AVOIDANCE_OBSTACLE_QUEUE_LENGTH = 10  # The number of obstacles to store at a time.
AVOIDANCE_MAX_SPEED_MPS = 0.6  # The max speed in meters per second to drive the rover. MUST MAKE SURE THIS IS ATTAINABLE WITH DRIVE SPEED POWER.

# Vision Parameters
MAX_DETECTION_ATTEMPTS = 50  # This should be about 1 second
ARUCO_FRAMES_DETECTED = 5  # ArUco Detection Occurrences
ARUCO_MARKER_BORDER_BITS = 1
ARUCO_ERROR_CORRECTION_RATE = 1
ARUCO_ENABLE_DISTANCE = 25
ARUCO_GOAL_DISTANCE_THRESH = (
    25  # The minimum distance from the goal waypoint before aruco detection os considered valid.
)
DISPLAY_TEST_MODE = False  # This will enable opening of OpenCV windows for vision detection live viewing.
ZED_X_OFFSET = 0.060325
ZED_Z_OFFSET = 0.000

# LiDAR maximum distance before we decide we would yeet off a cliff.
LIDAR_MAXIMUM = 250  # 2.5m to test early, need to determine actual value.

# Range at which we switch from GPS to optical tracking
VISION_RANGE = 0.007  # meters
MIN_OBSTACLE_PIXEL_AREA = 400  # minimum contour area in pixels of detected obstacle
DEPTH_STEP_SIZE = 0.5  # Depth in meters that each segment of obstacle detection will run on
NUM_DEPTH_SEGMENTS = 3  # Number of segments of depth map to actually run contour detection on


Coordinate = collections.namedtuple("Coordinate", ["lat", "lon"])

# RoveComm Numerical Values used to map events to enumerations to send over rovecomm
rovecomm_event_list = open("core/rovecomm_values.json", "r").read()
rovecomm_event_list = json.loads(rovecomm_event_list)

# RoveComm manifest
manifest = {}

# Outgoing communication ports
UDP_OUTGOING_PORT = 0000
TCP_OUTGOING_PORT = 11111


class ApproachState(Enum):
    APPROACHING = 1
    CLOSE_ENOUGH = 2
    PAST_GOAL = 3


class OperationState(IntEnum):
    TELEOP = 0
    AUTONOMY = 1
    REACHED_MARKER = 2
