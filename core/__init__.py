from core.rovecomm import RoveComm, RoveCommPacket
import core.constants
import core.notify
from core.waypoints import WaypointHandler
from core.manifest import *
import core.states
from core.states import AutonomyEvents

# RoveComm node, must be declared before it can be used.
rovecomm_node: RoveComm

# Waypoint handler
waypoint_handler: WaypointHandler = WaypointHandler()
