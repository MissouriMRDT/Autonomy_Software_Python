import drivers.rovecomm
import algorithms.objecttracking
import autonomy
import algorithms.geomath
import drivers.motorsRoveComm
import drivers.navboard_gps
import time
import queue
import struct
import drivers.Magnetometer
import logging

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
# Range at which we switch from GPS to optical tracking
VISION_ENABLED = False
VISION_RANGE = 0.007  # Kilometers

# Local navigation parameters
WIDTH           = 640.0  # pixels
FIELD_OF_VIEW   = 40.0   # degrees
TARGET_DISTANCE = 0.4    # meters
RADIUS          = .063   # meters
SCALING_FACTOR  = 10.0   # pixel-meters
POWER           = 35     # Percent

# RoveComm autonomy control DataIDs
ENABLE_AUTONOMY = 2576
DISABLE_AUTONOMY = 2577
ADD_WAYPOINT = 2578
CLEAR_WAYPOINTS = 2579
WAYPOINT_REACHED = 2580

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
rovecomm_node = drivers.rovecomm.RoveComm()

gps = drivers.navboard_gps.GPS(rovecomm_node)
compass = drivers.Magnetometer.Compass(rovecomm_node)
motors = drivers.motorsRoveComm.Motors()
tracker = algorithms.objecttracking.ObjectTracker()

autonomy_algorithm = autonomy.Autonomy(gps, compass, motors)

# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = struct.unpack("<dd", packet_contents)
    waypoint = algorithms.geomath.Coordinate(latitude, longitude)
    waypoints.put(waypoint)
    logging.info("Added waypoint %s" % (waypoint,))


def enable_autonomy(packet_contents):
    global autonomy_enabled
    autonomy_enabled = True
    logging.info("Autonomy Enabled")
    motors.enable()

def disable_autonomy(packet_contents):
    global autonomy_enabled
    global motors
    autonomy_enabled = False
    logging.info("Autonomy Disabled")
    motors.disable()


def clear_waypoint_handler(packet_contents):
    global waypoints
    global state
    logging.info("Waypoints Cleared")
    waypoints = queue.Queue()
    motors.disable()
    state = "idle"

def do_nothing(packet_contents):
    pass

rovecomm_node.callbacks[ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[ADD_WAYPOINT] = add_waypoint_handler
rovecomm_node.callbacks[CLEAR_WAYPOINTS] = clear_waypoint_handler
#debug
rovecomm_node.callbacks[1313] = do_nothing
rovecomm_node.callbacks[1314] = do_nothing
rovecomm_node.callbacks[1315] = do_nothing
rovecomm_node.callbacks[1296] = do_nothing

state = 'idle'
current_goal = None
while state != 'shutdown':
    if autonomy_enabled:
        if state == 'idle':
            time.sleep(0.2)
            if not waypoints.empty():
                state = 'gps_navigate'
                current_goal = waypoints.get_nowait()
                autonomy_algorithm.setWaypoint(current_goal)
            else:
                pass

        elif state == 'gps_navigate':
            reached_goal = autonomy_algorithm.update_controls()
            time.sleep(0.01)
            
            if reached_goal:
                state = 'waypoint_reached'
            elif VISION_ENABLED and autonomy_algorithm.distance_to_goal < VISION_RANGE and waypoints.empty():
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
                angle_to_ball = FIELD_OF_VIEW * ((center[0] - (WIDTH / 2)) / WIDTH)
                distance = SCALING_FACTOR / radius
                logging.info("Distance: %f" % distance)
                if distance > TARGET_DISTANCE:
                    logging.info("Moving forward: %f" % angle_to_ball)
                    motors.move(POWER, angle_to_ball)
                if distance <= TARGET_DISTANCE:
                    state = 'waypoint_reached'
                    motors.move(-POWER, angle_to_ball)
            else:
                logging.info("Visual lock lost")
                state = 'gps_navigate'

        elif state == 'waypoint_reached':
            rovecomm_node.send(WAYPOINT_REACHED, contents="")
            motors.disable()
            time.sleep(1)
            if not waypoints.empty():
                current_goal = waypoints.get_nowait()
                autonomy_algorithm.setWaypoint(current_goal)
                logging.info('Moving to next waypoint')
                motors.enable()
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
 `