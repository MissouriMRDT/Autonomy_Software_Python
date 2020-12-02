<<<<<<< HEAD
import core
=======
import logging
>>>>>>> feature/nav_logging
from core.state import RoverState
from enum import Enum


class StateMachine(object):
    """
    The state machine "handler" which will deal with switching between various states
    """

    def __init__(self):
        self.state = Idle()

        # Set shutdown and start flags
        self.enable_flag = False
        self.disable_flag = False

        # Add callbacks for autonomy controls
        core.rovecomm.set_callback(core.ENABLE_AUTONOMY_ID, self.enable)
        core.rovecomm.set_callback(core.DISABLE_AUTONOMY_ID, self.disable)

    def enable(self):
        self.enable_flag = True

    def disable(self):
        self.disable_flag = True

    async def run(self):
        # Handle transitions into Idle and Navigating states
        if self.enable_flag is True:
            self.state = Navigating()
            self.enable_flag = False

        elif self.disable_flag is True:
            self.state = Idle()
            self.disable_flag = False

        # Run the current state
        self.state = self.state.run()


class Idle(RoverState):
    """
    This is the default state for the state machine. In this state the program does nothing explicit.
    Its singular purpose is to keep the python program running to receive and transmit rovecomm commands from base station that configure the next leg’s settings and confirm them.
    """

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
    """
    The goal of this state is to navigate to the GPS coordinates provided by base station in succession, the last of which is the coordinate provided by the judges for that leg of the task.
    Coordinates before the last are simply the operators in base station’s best guess of the best path for the rover due to terrain identified on RED’s map.
    """

    def handle_event(self, event, then=None):

        if event == AutonomyEvents.REACHED_GPS_COORDINATE:
            return Searching()
        elif event == AutonomyEvents.ABORT:
            return Shutdown()

        if then:
            then()  # callback

        return self


class Searching(RoverState):
    """
    The searching state’s goal is to drive the rover in an ever expanding Archimedean spiral, searching for the AR Tag.
    The spiral type was chosen because of it’s fixed distance between each rotation’s path.
    """

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
    """
    Within approaching marker, the rover explicitly follows the spotted marker until it reaches an acceptable distance from the rover, or loses sight of it.
    """

    def handle_event(self, event, then=None):

        if event == AutonomyEvents.REACHED_MARKER:
            return Idle()
        elif event == AutonomyEvents.MARKER_UNSEEN:
            return Searching()
        elif event == AutonomyEvents.ABORT:
            return Shutdown()
        elif event == AutonomyEvents.OBSTACLE_AVOIDANCE:
            return ObstacleAvoidance()

        if then:
            then()  # callback

        return self


class Shutdown(RoverState):
    """
    Within any state except for Idle, all implement ABORT as a throwable when a stop message is received from base station.
    All abort throws are sent to shutdown to stop the rover and resolve any loose ends that may arise from the navigation states.
    """

    def handle_event(self, event, then=None):

        if event == AutonomyEvents.RESTART:
            return Idle()

        if then:
            then()  # callback

        return self


class ObstacleAvoidance(RoverState):
    """
    This state provides navigation around obstacles found in the field.
    """

    def handle_event(self, event, then=None):

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
    OBSTACLE_AVOIDANCE = 10
    END_OBSTACLE_AVOIDANCE = 11