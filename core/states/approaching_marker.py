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

    def on_event(self, event) -> RoverState:
        """
        Defines all transitions between states based on events
        """
        if event == core.AutonomyEvents.REACHED_MARKER:
            return core.states.Idle()

        elif event == core.AutonomyEvents.START:
            return self

        elif event == core.AutonomyEvents.ABORT:
            return core.states.Idle()

        else:
            self.logger.error(f"Unexpected event {event} for state {self}")
            return core.states.Idle()

    async def run(self) -> RoverState:
        # Call AR Tag tracking code to find position and size of AR Tag
        # TODO: hook this up to actual tracking code
        ball_in_frame = True
        center = 0
        radius = 1

        if ball_in_frame:
            (left, right), distance = algorithms.follow_marker.drive_to_marker(75, center, radius)
            self.logger.debug("Ball in frame")
            if distance < 0.5:
                interfaces.drive_board.send_drive(0, 0)

                self.logger.info("Reached Marker")

                # TODO: Add support for

                # core.notify.notify_finish()
                return self.on_event(core.AutonomyEvents.REACHED_MARKER)
            else:
                print("Driving To: " + str(left) + ", " + str(right))
                Logger.write_line(
                    time.strftime("%H%M%S")
                    + " ApproachingMarker: Not reached marker, driving to "
                    + str(left)
                    + ","
                    + str(right)
                )
                rovecomm_node.write(drive.send_drive(left, right))

        else:  # commented for testing, remove the commenting for running.
            state_switcher.handle_event(rs.AutonomyEvents.MARKER_UNSEEN, rs.ApproachingMarker())
            Logger.write_line(
                time.strftime("%H%M%S") + " ApproachingMarker: lost sight of marker, returning to Searching()"
            )
            print("Throwing MARKER_UNSEEN")
        return self
