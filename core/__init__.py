from core.constants import *
from core.states.state_machine import StateMachine
from core.rovecomm import RoveComm, RoveCommPacket
import core.notify
from core.waypoints import WaypointHandler
import core.states as states
from core.states import AutonomyEvents
import core.vision as vision
import json

# RoveComm node, must be declared before it can be used.
rovecomm_node: RoveComm

# Waypoint handler
waypoint_handler: WaypointHandler


def setup(type="REGULAR"):
    """
    Sets up any core handlers (excluding RoveComm)
    """
    # load the manifest depending on type
    if type == "REGULAR":
        core.UDP_OUTGOING_PORT = 11000
        core.manifest = open("core/manifest.json", "r").read()
        core.manifest = json.loads(core.manifest)
    elif type == "SIM":
        core.UDP_OUTGOING_PORT = 11001
        core.manifest = open("core/manifest_sim.json", "r").read()
        core.manifest = json.loads(core.manifest)

    # Initialize basic handlers
    core.waypoint_handler = WaypointHandler()
    core.states.state_machine = StateMachine()
