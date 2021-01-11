from core.states.state_machine import StateMachine
from core.rovecomm import RoveComm, RoveCommPacket
import core.constants
import core.notify
from core.waypoints import WaypointHandler
from core.manifest import *
import core.states
from core.states import AutonomyEvents
import core.vision

# RoveComm node, must be declared before it can be used.
rovecomm_node: RoveComm

# Waypoint handler
waypoint_handler: WaypointHandler


def setup():
    """
    Sets up any core handlers (excluding RoveComm)
    """
    core.waypoint_handler = WaypointHandler()
    core.states.state_machine = StateMachine()
