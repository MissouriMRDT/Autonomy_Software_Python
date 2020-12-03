from states.state import RoverState
from states.rover_states import StateMachine
from states.idle import Idle
from states.search_pattern import SearchPattern
from states.navigating import Navigating

# State Machine handler, takes care of running states and enabling/disabling autonomy
state_machine: StateMachine