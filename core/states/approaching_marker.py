import asyncio
import core
import interfaces
import algorithms
from core.states import RoverState


class ApproachingMarker(RoverState):
    """
    The goal of this state is to navigate to the GPS coordinates provided by base
    station in succession, the last of which is the coordinate provided by the judges
    for that leg of the task. Coordinates before the last are simply the operators in
    base station’s best guess of the best path for the rover due to terrain identified
    on RED’s map.
    """

    def start(self):
        loop = asyncio.get_event_loop()

        # Schedule AR Tag detection
        self.ar_tag_task = loop.create_task()

    def exit(self):
        # Cancel all state specific tasks
        self.ar_tag_task.cancel()

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        state: RoverState = None

        if event == core.AutonomyEvents.REACHED_MARKER:
            state = core.states.Idle()

        elif event == core.AutonomyEvents.START:
            state = self

        elif event == core.AutonomyEvents.MARKER_UNSEEN:
            state = core.states.SearchPattern()

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
        # Call AR Tag tracking code to find position and size of AR Tag
        # TODO: hook this up to actual tracking code
        ball_in_frame = True
        center = 0
        radius = 1

        if ball_in_frame:
            (left, right), distance = algorithms.follow_marker.drive_to_marker(75, center, radius)
            self.logger.info("Marker in frame")

            if distance < 0.5:
                interfaces.drive_board.stop()

                self.logger.info("Reached Marker")

                # TODO: Add support for notifying (with LEDs) that we have reached marker
                # core.notify.notify_finish()
                return self.on_event(core.AutonomyEvents.REACHED_MARKER)
            else:
                self.logger.debug(f"Driving to target with speeds: ({left}, {right})")
                interfaces.drive_board.send_drive(left, right)

        else:
            self.logger.info("Lost sign of marker, returning to Search Pattern")
            return self.on_event(core.AutonomyEvents.MARKER_UNSEEN)

        return self
