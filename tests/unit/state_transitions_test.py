from math import nan
import pandas as pd
import core.states


"""
UNIT TEST
FILES: core/states/

This file provides unit tests for state transitions for all states in core/states/

If issues are encountered:
Double check to make sure docs/state_machine_matrix.csv is up-to-date.

Added a new event? Make sure to update the state machine matrix csv. If the number of
events is not correct, it will cause issues.
"""

df = None
autonomy_events = None
path_to_csv = "docs/state_machine_matrix.csv"


def state_string_to_class(state_str: str) -> core.states.RoverState:
    """
    Utility function to return a RoverState based on string in csv matrix
    Parameters:
        state_str (string)

    Returns:
        state (RoverState)
    """
    if state_str == "" or pd.isnull(state_str):
        # Blank transitions are unexpected, and therefore cause the transition back to Idle
        return core.states.Idle()
    else:
        state = getattr(core.states, state_str)
        return state()


def setup_module(module):
    global df, autonomy_events

    df = pd.read_csv("docs/state_machine_matrix.csv")
    autonomy_events = df["Events"].unique()


def test_idle_transitions():
    # All the transitions for Idle() defined in csv
    idle_transitions = df["Idle"]

    # Test each of the possible transitions for the IDle state individually
    for i in range(len(idle_transitions)):
        # Default state to Idle
        state = core.states.Idle()
        # Assert that the transition that occurs is the same as specified in the csv
        assert state.on_event(core.states.AutonomyEvents(i + 1)) == state_string_to_class(idle_transitions[i])


def test_navigating_transitions():
    # All the transitions for Navigating() defined in csv
    navigating_transitions = df["Navigating"]

    # Test each of the possible transitions for the Navigating state individually
    for i in range(len(navigating_transitions)):
        # Default state to Navigating
        state = core.states.Navigating()
        # Assert that the transition that occurs is the same as specified in the csv
        assert state.on_event(core.states.AutonomyEvents(i + 1)) == state_string_to_class(navigating_transitions[i])


def test_search_pattern_transitions():
    # All the transitions for SearchPattern() defined in csv
    search_pattern_transitions = df["SearchPattern"]

    # Test each of the possible transitions for the Navigating state individually
    for i in range(len(search_pattern_transitions)):
        # Default state to Search Pattern
        state = core.states.SearchPattern()
        # Assert that the transition that occurs is the same as specified in the csv
        assert state.on_event(core.states.AutonomyEvents(i + 1)) == state_string_to_class(search_pattern_transitions[i])


def test_approaching_marker_transitions():
    # All the transitions for ApproachingMarker() defined in csv
    approaching_marker_transitions = df["ApproachingMarker"]

    # Test each of the possible transitions for the Navigating state individually
    for i in range(len(approaching_marker_transitions)):
        # Default state to ApproachingMarker
        state = core.states.ApproachingMarker()
        # Assert that the transition that occurs is the same as specified in the csv
        assert state.on_event(core.states.AutonomyEvents(i + 1)) == state_string_to_class(
            approaching_marker_transitions[i]
        )


def test_avoidance_transitions():
    # All the transitions for Avoidance() defined in csv
    avoidance_transitions = df["Avoidance"]

    # Test each of the possible transitions for the Navigating state individually
    for i in range(len(avoidance_transitions)):
        # Default state to ApproachingMarker
        state = core.states.Avoidance()
        # Assert that the transition that occurs is the same as specified in the csv
        assert state.on_event(core.states.AutonomyEvents(i + 1)) == state_string_to_class(avoidance_transitions[i])
