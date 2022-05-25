import core
import interfaces
import asyncio
from core.states import RoverState
import time


class SearchPattern(RoverState):
    """
    The searching state’s goal is to drive the rover in an ever expanding Archimedean spiral, searching for the AR Tag.
    The spiral type was chosen because of it’s fixed distance between each rotation’s path.
    """

    def start(self):
        global t1
        t1 = time.time()

    def exit(self):
        # Cancel all state specific tasks
        pass

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        state: RoverState = None

        if event == core.AutonomyEvents.MARKER_SEEN:
            state = core.states.ApproachingMarker()

        elif event == core.AutonomyEvents.GATE_SEEN:
            state = core.states.ApproachingGate()

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.SEARCH_FAILED:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.ABORT:
            state = core.states.Idle()

        else:
            self.logger.error(f"Unexpected event {event} for state {self}")
            state = core.states.Idle()

        # Call exit() if we are not staying the same state
        if state != self:
            self.exit()

        # Return the state appropriate for the event
        return state


    async def run(self) -> RoverState:
        
        t2 = time.time()

        if core.vision.ar_tag_detector.is_gate():
            core.waypoint_handler.gps_data.leg_type = "GATE"
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(0.1)

            self.logger.info("Search Pattern: Gate seen")
            return self.on_event(core.AutonomyEvents.GATE_SEEN)

        elif core.vision.ar_tag_detector.is_marker():
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(0.1)

            self.logger.info("Search Pattern: Marker seen")
            return self.on_event(core.AutonomyEvents.MARKER_SEEN)

        await asyncio.sleep(0.1)
        seconds_elapsed = t2 - t1
        left = int(core.MAX_DRIVE_POWER - (seconds_elapsed * core.INCREASE_INCREMENT))
        right = int(core.MIN_DRIVE_POWER + (seconds_elapsed * core.DECREASE_INCREMENT))

        print(f"Search Pattern: Driving at ({left}, {right})")
        interfaces.drive_board.send_drive(left, right)
            
        return self