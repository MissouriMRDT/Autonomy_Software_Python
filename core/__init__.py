from core.constants import *
from core.states.state_machine import StateMachine
from core.rovecomm import RoveComm, RoveCommPacket
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
    Sets up any core handlers (excluding RoveComm)
    """
    # load the manifest depending on type
    if type == "REGULAR":
        this.UDP_OUTGOING_PORT = 11000
        this.manifest = open("core/manifest.json", "r").read()
        this.manifest = json.loads(this.manifest)
        this.manifest = this.manifest["RovecommManifest"]
    elif type == "SIM":
        this.UDP_OUTGOING_PORT = 11001
        this.manifest = open("core/manifest.json", "r").read()
        this.manifest = json.loads(this.manifest)
        this.manifest = this.manifest["RovecommManifest"]

        # For simulation, all Ips are localhost
        for board in this.manifest:
            this.manifest[board]["Ip"] = "127.0.0.1"

    # Initialize basic handlers
    this.waypoint_handler = WaypointHandler()
    this.states.state_machine = StateMachine()
