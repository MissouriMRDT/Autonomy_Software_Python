from state import RoverState
from enum import Enum


class StateSwitcher(object):

    def __init__(self):
        self.state = Idle()  # default state

    def handle_event(self, event, then=None):
        self.state = self.state.handle_event(event)
        if then:
            then()  # callback


class Idle(RoverState):

    def handle_event(self, event, then=None):
        if event == AutonomyEvents.START:
            return Navigating()
        elif event == AutonomyEvents.ALL_MARKERS_REACHED:
            return Idle()
        elif event == AutonomyEvents.ABORT:
            return Shutdown()

        if then:
            then()  # callback

        return self


class Navigating(RoverState):

    def handle_event(self, event, then=None):

        if event == AutonomyEvents.REACHED_GPS_COORDINATE:
            return Searching()
        elif event == AutonomyEvents.ABORT:
            return Shutdown()

        if then:
            then()  # callback

        return self


class Searching(RoverState):

    def handle_event(self, event, then=None):

        if event == AutonomyEvents.MARKER_SIGHTED:
            return ApproachingMarker()
        elif event == AutonomyEvents.SEARCH_FAILED:
            return Navigating()
        elif event == AutonomyEvents.ABORT:
            return Shutdown()

        if then:
            then()  # callback

        return self


class ApproachingMarker(RoverState):

    def handle_event(self, event, then=None):

        if event == AutonomyEvents.REACHED_MARKER:
            return Navigating()
        elif event == AutonomyEvents.MARKER_UNSEEN:
            return Searching()
        elif event == AutonomyEvents.ABORT:
            return Shutdown()

        if then:
            then()  # callback

        return self


class Shutdown(RoverState):

    def handle_event(self, event, then=None):

        if event == AutonomyEvents.RESTART:
            return Idle()

        if then:
            then()  # callback

        return self


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
