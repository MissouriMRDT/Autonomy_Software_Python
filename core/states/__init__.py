#
# Mars Rover Design Team
# __init__.py
#
# Created on Dec 02, 2020
# Updated on Aug 21, 2022
#

from core.states.state import RoverState
from core.states.state_machine import StateMachine
from core.states.idle import Idle
from core.states.search_pattern import SearchPattern
from core.states.old_navigating import Navigating
from core.states.approaching_marker import ApproachingMarker
from core.states.approaching_gate import ApproachingGate
from core.states.avoidance import Avoidance
from enum import Enum

# State Machine handler, takes care of running states and enabling/disabling autonomy
state_machine: StateMachine


class AutonomyEvents(Enum):
    START = 1
    REACHED_GPS_COORDINATE = 2
    MARKER_SEEN = 3
    GATE_SEEN = 4
    MARKER_UNSEEN = 5
    SEARCH_FAILED = 6
    REACHED_MARKER = 7
    ALL_MARKERS_REACHED = 8
    ABORT = 9
    RESTART = 10
    OBSTACLE_AVOIDANCE = 11
    END_OBSTACLE_AVOIDANCE = 12
    NO_WAYPOINT = 13
    NEW_WAYPOINT = 14


StateMapping = {
    "Idle": 0,
    "Navigating": 1,
    "SearchPattern": 2,
    "ApproachingMarker": 3,
    "ApproachingGate": 4,
    "Avoidance": 5,
}
