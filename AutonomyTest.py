import drivers.rovecomm
#import algorithms.objecttracking
#import algorithms.autonomy
#import drivers.motors_rovecomm
import drivers.navboard_gps
import time
import Queue
import struct
import drivers.Magnetometer

# ---------------------------------------------------------
# Configuration
# ---------------------------------------------------------
# Range at which we switch from GPS to optical tracking
VISION_RANGE = 7  # Meters

# RoveComm autonomy control DataIDs
ENABLE_AUTONOMY = 2576
DISABLE_AUTONOMY = 2577
ADD_WAYPOINT = 2578
CLEAR_WAYPOINTS = 2579
WAYPOINT_REACHED = 2590

# -------------
# State
# -------------
waypoints = Queue.Queue()
autonomy_enabled = False

# ---------------------------------------------------------
# Connect to hardware
#
# ---------------------------------------------------------
rovecomm_node = drivers.rovecomm.RoveComm()

gps = drivers.navboard_gps.GPS(rovecomm_node)
compass = drivers.Magnetometer.Compass(rovecomm_node)
#motors = drivers.motors_rovecomm.Motors(rovecomm_node)

#autonomy_algorithm = algorithms.autonomy.Autonomy(gps, compass, motors)

# Assign callbacks for incoming messages
def add_waypoint_handler(packet_contents):
    latitude, longitude = struct.unpack(">dd", packet_contents)
    waypoint = (latitude, longitude)
    waypoints.put(waypoint)


def enable_autonomy(packet_contents):
    global autonomy_enabled
    autonomy_enabled = True


def disable_autonomy(packet_contents):
    global autonomy_enabled
    autonomy_enabled = False


def clear_waypoint_handler(packet_contents):
    global waypoints
    waypoints = Queue.Queue()


rovecomm_node.callbacks[ENABLE_AUTONOMY] = enable_autonomy
rovecomm_node.callbacks[DISABLE_AUTONOMY] = disable_autonomy
rovecomm_node.callbacks[ADD_WAYPOINT] = add_waypoint_handler
rovecomm_node.callbacks[CLEAR_WAYPOINTS] = clear_waypoint_handler

state = 'idle'
current_goal = None
while state != 'shutdown':
    if autonomy_enabled:
        if state == 'idle':
            if not waypoints.empty():
                state = 'gps_navigate'
                current_goal = waypoints.get()
            else:
                pass

        elif state == 'gps_navigate':
            print(current_goal)
            time.sleep(3)
            state = 'vision_navigate'
            # if distance to current_goal < VISION_RANGE:
            #    if ball visible in camera:
            #        state = 'vision_navigate'
            # else:
            #    state = 'gps_navigate'

        elif state == 'vision_navigate':
            # Update machine vision algorithm here
            print(current_goal)
            time.sleep(2)
            state = 'waypoint_reached'
            # if goal reached:
            #    state = 'waypoint reached'
            # elif lost sight of ball:
            #    state = 'gps_navigate'
            # else:
            #    state = 'vision_navigate'

        elif state == 'waypoint reached':
            rovecomm_node.send(WAYPOINT_REACHED, contents="")
            node = waypoints.get_nowait()
            state = 'idle'

        elif state == 'error':
            print("Call Owen for a good time")

        print("Current state: %s\t Current Goal: %s" % (state, current_goal))
    else:
        pass
