from core.constants import *
from core.states.state_machine import StateMachine
from core.rovecomm_module.rovecomm import RoveComm, RoveCommPacket, get_manifest
from core.waypoints import WaypointHandler
import core.states as states
from core.states import AutonomyEvents
import core.vision as vision
import json
import sys

# reference to self
this = sys.modules[__name__]

# RoveComm node, must be declared before it can be used.
rovecomm_node: RoveComm

# Waypoint handler
waypoint_handler: WaypointHandler


def setup(type="REGULAR"):
    """
    Sets up any core handlers
    """
    # load the manifest
    this.manifest = get_manifest()

    # IPs and ports depend on type
    if type == "REGULAR":
        this.UDP_OUTGOING_PORT = 11000
    elif type == "SIM":
        this.UDP_OUTGOING_PORT = 11001

        # For simulation, all Ips are localhost
        for board in this.manifest:
            this.manifest[board]["Ip"] = "127.0.0.1"

    # Initialize basic handlers
    this.waypoint_handler = WaypointHandler()
    this.states.state_machine = StateMachine()
