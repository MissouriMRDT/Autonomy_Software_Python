import drivers.rovecomm
import time
import binascii
import Queue
import struct

# Range at which we switch from GPS to optical tracking
VISION_RANGE = 7 # Meters

ENABLE_AUTONOMY = 2576
DISABLE_AUTONOMY = 2577
ADD_WAYPOINT = 2578
CLEAR_WAYPOINTS = 2579
WAYPOINT_REACHED = 2590

rove_comm = drivers.rovecomm.RoveComm()
waypoints = Queue.Queue()

autonomy_enabled = False

def addWaypointHandler(packet_contents):

    latitude, longitude = struct.unpack(">dd", packet_contents)
    waypoint = (latitude, longitude)
    waypoints.put(waypoint)

def enableAutonomy(packet_contents):
    autonomy_enabled = True

def disableAutonomy(packet_contents):
    autonomy_enabled = False

def clearWaypointHandler(packet_contents):
    waypoints = Queue.Queue()

rove_comm.callbacks[ENABLE_AUTONOMY] = enableAutonomy
rove_comm.callbacks[DISABLE_AUTONOMY] = disableAutonomy
rove_comm.callbacks[ADD_WAYPOINT] = addWaypointHandler
rove_comm.callbacks[CLEAR_WAYPOINTS] = clearWaypointHandler

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
            #if distance to current_goal < VISION_RANGE:
            #    if ball visible in camera:
            #        state = 'vision_navigate'
            #else:
            #    state = 'gps_navigate'

        elif state == 'vision_navigate':
            # Update machine vision algorithm here
            print(current_goal)
            time.sleep(2)
            state = 'waypoint_reached'
            #if goal reached:
            #    state = 'waypoint reached'
            #elif lost sight of ball:
            #    state = 'gps_navigate'
            #else:
            #    state = 'vision_navigate'

        elif state == 'waypoint reached':
            rove_comm.send(WAYPOINT_REACHED, contents="")
            state = 'idle'

        elif state == 'error':
            print("Call Owen for a good time")

        print("Current state: %s\t Current Goal: %s" % (state, current_goal))
    else:
        pass

