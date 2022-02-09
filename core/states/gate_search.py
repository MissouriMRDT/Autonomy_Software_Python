# AUTHORS: Donovan Bale, Aaron Dobsch, Duncan Truitt, Jacob Vandorin

#Organize and Cleanup imports
import asyncio
from algorithms import geomath
from core.vision import obstacle_avoidance
from core.waypoints import WaypointHandler
import core
import interfaces
import algorithms
from core.vision import ar_tag_detector
from core.states import RoverState


class GateSearch(RoverState):

    def start(self):
        pass

    def exit(self):
        # Cancel all state specific coroutines
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
        
        """
        Defines regular rover operation when under this state
        """
        
        # Get current GPS location of rover and ar_tag
        current = interfaces.nav_board.location()
        while(not core.vision.ar_tag_detector.is_gate() and not core.vision.ar_tag_detector.is_marker()):
            await asyncio.sleep(core.EVENT_LOOP_DELAY)
        bearing, distance = ar_tag_detector.get_tags()[0].angle, ar_tag_detector.get_tags()[0].distance
        tag_latitude, tag_longitude = geomath.reverse_haversine(bearing, distance, current[0], current[1]) # *** Reverse haversince is in km ***


        # Compute lat and long to approach exactly 4m from tag, then append to waypoint deque *** THIS NEEDS TO BE DEGRADATED
        tag_approach_lat, tag_approach_long = geomath.reverse_haversine(bearing, distance - 0.004, current[0], current[1])    
        waypoint = core.Coordinate(tag_approach_lat, tag_approach_long)
        core.waypoint_handler.waypoints.appendleft(("POSITION", waypoint)) #This needs to be modified a bit MAYBE
        self.logger.info(f"Added Position Waypoint to Front of Queue: lat ({tag_approach_lat}), lon ({tag_approach_long})") # Consider updating goal coordinate


        # Compute waypoints that approximate circle and append to list
        circle_points = [waypoint]
        circle_points += self.compute_circle_waypoints(tag_latitude, tag_longitude, 4, 8) #2nd and third parameters are radius (in m) and increments (number of waypoints)
        self.logger.info(f"Added position waypoints that go in circle around AR tag: lat ({tag_latitude}), lon ({tag_longitude})")
        start = core.Coordinate(interfaces.nav_board.location()[0], interfaces.nav_board.location()[1])


        # Iterate through circle_points and check for gate while moving
        for point in circle_points:
            while (
                algorithms.gps_navigate.get_approach_status(
                    core.Coordinate(point[0], point[1]), interfaces.nav_board.location(), start, 0.5
                )
                == core.ApproachState.APPROACHING
            ):
                # Potential Issue Here -- The rever MUST see both tags in the same image for this function to return true
                # Also note that no obstacle detection occurs here
                if core.vision.ar_tag_detector.is_gate():
                    return core.states.ApproachingGate()

                self.logger.info(f"Driving towards: Lat: {point[0]}, Lon: {point[1]}")
                left, right = algorithms.gps_navigate.calculate_move(
                    point,
                    interfaces.nav_board.location(),
                    start,
                    250,
                )

                self.logger.debug(f"Diving at speeds: Left: {left} Right: {right}")

                interfaces.drive_board.send_drive(left, right)
                await asyncio.sleep(core.EVENT_LOOP_DELAY)
            interfaces.drive_board.stop()

        #turn 90 left
        speed = 100 #Maybe change this?
        speed1, speed2 = interfaces.drive_board.calculate_move(speed, -90)
        interfaces.drive_board.send_drive(speed1, speed2)

        return core.states.ApproachingMarker()




    def compute_circle_waypoints(self, tag_lat, tag_long, circle_radius, increments):

        '''
        
        Returns a tuple of waypoints that approximate a circle around the target.

        *** STILL A WORK IN PROGRESS AND UNTESTED ***

        Parameters:
        --------
        tag_lat - GPS latitude coord of ar tag
        tag_long - GPS longitude coord of ar tag
        circle_radius - Radius of the circle that points will be around ar tag (*** in meters ***)
        increments - number of points for the approximated circle that will be returned

        Returns:
        --------
        tuple of core.Coordinate objects containing GPS coords for each point in the circle

        '''

        circle_radius /= 1000 # Convert circle radius to km


        #compute bearing and distance to tag
        current = interfaces.nav_board.location()
        bearing, distance = geomath.haversine(tag_lat, tag_long, current[0], current[1])

        # compute angle differentials
        angle_diff = 360 / increments
        circle_waypoints = []

        #compute waypoints to make rover go in a circle around the tag
        for i in range(increments):
            
            new_angle = bearing + angle_diff * (i+1) # *** Need to verify whether reverse_haversine can handle angles outside 0-360 range ***
            point_lat, point_long = geomath.reverse_haversine(new_angle, circle_radius, tag_lat, tag_long)
            circle_waypoints.append(core.Coordinate(point_lat, point_long))
        
        return circle_waypoints



# ---- QUESTIONS / ISSUES ---- #
#
# 1. The way that the is_gate() detection works is that it looks to see if TWO AR tags are on the screen at once.
# 2. How does the state loop?
# 3. Set waypoint goal to ar tag location? (Maybe done already)
# 4. Figure out where to stow run code that should only execute once.
# 5. Angle given by camera may not reflect haversine angle
#
#
