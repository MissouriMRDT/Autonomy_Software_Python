from core.states.state import RoverState
from core.states.rover_states import StateMachine
from core.states.idle import Idle
from core.states.search_pattern import SearchPattern
from core.states.navigating import Navigating
from core.states.approaching_marker import ApproachingMarker
from enum import Enum

# State Machine handler, takes care of running states and enabling/disabling autonomy
state_machine: StateMachine


class AutonomyEvents(Enum):
    START = 1
    REACHED_GPS_COORDINATE = 2
    MARKER_SIGHTED = 3
    SEARCH_FAILED = 4
    MARKER_UNSEEN = 5
    REACHED_MARKER = 6
    ALL_MARKERS_REACHED = 7
    ABORT = 8
    RESTART = 9
    OBSTACLE_AVOIDANCE = 10
    END_OBSTACLE_AVOIDANCE = 11
    NO_WAYPOINT = 12
    NEW_WAYPOINT = 13
