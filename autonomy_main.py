import time
import queue
import struct
import logging

from drivers.mag.compass import Compass
from drivers.gps.gpsNavboard import GPS
from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard
from drivers.notify import Notify

from algorithms.objecttracking import ObjectTracker
from gpsNavigate import GPSNavigate
import algorithms.geomath as geomath
import constants
from constants import DataID
from rover_states import StateSwitcher, AutonomyEvents

# Configuration
VISION_ENABLED = False

logging.basicConfig(filename='autonomy.log',
                    format='%(asctime)s %(name)s\t: %(levelname)s\t%(message)s',
                    level=logging.INFO)
console = logging.StreamHandler()
formatter = logging.Formatter('%(name)s\t: %(levelname)s\t%(message)s')
console.setFormatter(formatter)
logging.getLogger('').addHandler(console)

# -------------
# State
# -------------
waypoints = queue.Queue()
autonomy_enabled = False

# ---------------------------------------------------------
# Connect to hardware
#
# ---------------------------------------------------------
rovecomm_node = RoveComm()

gps = GPS(rovecomm_node)
compass = Compass(rovecomm_node)
drive = DriveBoard(rovecomm_node)
notify = Notify(rovecomm_node)

tracker = ObjectTracker()
# lidar = LiDAR(rovecomm_node)
gpsNavigation = GPSNavigate(gps, compass, drive, """lidar""")


# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = struct.unpack("<dd", packet_contents)
    waypoint = geomath.Coordinate(latitude, longitude)
    waypoints.put(waypoint)
    logging.info("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    global autonomy_enabled
    autonomy_enabled = True
    logging.info("Autonomy Enabled")
    drive.enable()


def disable_autonomy(packet_contents):
    global autonomy_enabled
    global drive
    autonomy_enabled = False
    logging.info("Autonomy Disabled")
    drive.disable()


def clear_waypoint_handler(packet_contents):
    global waypoints
    global state
    logging.info("Waypoints Cleared")
    waypoints = queue.Queue()
    drive.disable()
    state = "idle"


def do_nothing(packet_contents):
    pass


rovecomm_node.callbacks[DataID.ENABLE_AUTONOMY.value] = enable_autonomy
rovecomm_node.callbacks[DataID.DISABLE_AUTONOMY.value] = disable_autonomy
rovecomm_node.callbacks[DataID.ADD_WAYPOINT.value] = add_waypoint_handler
rovecomm_node.callbacks[DataID.CLEAR_WAYPOINTS.value] = clear_waypoint_handler


stateSwitcher = StateSwitcher()

current_goal = None


def test_func():
    print("callback successful")


stateSwitcher.handle_event(AutonomyEvents.START, callback=test_func)

state = "shutdown"
while state != 'shutdown':
    if autonomy_enabled:
        if state == 'idle':
            time.sleep(0.2)
            if not waypoints.empty():
                state = 'gps_navigate'
                current_goal = waypoints.get_nowait()
                gpsNavigation.setWaypoint(current_goal)
            else:
                pass

        elif state == 'gps_navigate':
            reached_goal = gpsNavigation.update_controls()
            time.sleep(0.01)
            if reached_goal:
                state = 'waypoint_reached'
            elif VISION_ENABLED and gpsNavigation.distance_to_goal < constants.VISION_RANGE and waypoints.empty():
                logging.info('In vision range, searching')
                ball_in_frame, center, radius = tracker.track_ball()
                if ball_in_frame:
                    logging.info('Ball seen at %s with r=%i, locking on' % (center, radius))
                    state = 'vision_navigate'
                else:
                    state = 'gps_navigate'
            else:
                state = 'gps_navigate'

        elif state == 'vision_navigate':
            ball_in_frame, center, radius = tracker.track_ball()
            if ball_in_frame:
                angle_to_ball = constants.FIELD_OF_VIEW * ((center[0] - (constants.WIDTH / 2)) / constants.WIDTH)
                distance = constants.SCALING_FACTOR / radius
                logging.info("Distance: %f" % distance)
                if distance > constants.TARGET_DISTANCE:
                    logging.info("Moving forward: %f" % angle_to_ball)
                    drive.move(constants.POWER, angle_to_ball)
                if distance <= constants.TARGET_DISTANCE:
                    state = 'waypoint_reached'
                    drive.move(-constants.POWER, angle_to_ball)
            else:
                logging.info("Visual lock lost")
                state = 'gps_navigate'

        elif state == 'waypoint_reached':
            rovecomm_node.send(DataID.WAYPOINT_REACHED.value, contents="")
            notify.notifyFinish()
            drive.disable()
            time.sleep(1)
            if not waypoints.empty():
                current_goal = waypoints.get_nowait()
                gpsNavigation.setWaypoint(current_goal)
                logging.info('Moving to next waypoint')
                drive.enable()
                state = 'gps_navigate'
            else:
                logging.info('All waypoints reached, going into idle')
                current_goal = None
                state = 'idle'

        else:
            logging.error("Invalid state detected: %s" % state)

        logging.debug("Current state: %s\t Current Goal: %s" % (state, current_goal))
    else:
        time.sleep(0.2)
