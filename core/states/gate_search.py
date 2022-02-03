# AUTHORS: Donovan Bale, Aaron Dobsch, and Duncan Truitt

#Organize and Cleanup imports
import asyncio
from algorithms import geomath
from core.vision import obstacle_avoidance
from core.waypoints import WaypointHandler
import core
import interfaces
import algorithms
from vision import ar_tag_detector
from core.states import RoverState


class GateSearch(RoverState):


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
        bearing, distance = ar_tag_detector.get_tags()[0].angle, ar_tag_detector.get_tags()[0].distance
        tag_latitude, tag_longitude = geomath.reverse_haversine(bearing, distance, current[0], current[1]) # *** Reverse haversince is in km - Yuck


        # Compute lat and long to approach exactly 4m from tag, then append to waypoint deque
        tag_approach_lat, tag_approach_long = geomath.reverse_haversine(bearing, distance - 0.004, current[0], current[1])    
        waypoint = core.Coordinate(tag_approach_lat, tag_approach_long)
        core.waypoint_handler.waypoints.appendleft(("POSITION", waypoint)) #This needs to be modified a bit MAYBE
        self.logger.info(f"Added Position Waypoint to Front of Queue: lat ({tag_approach_latitude}), lon ({tag_approach_longitude})") # Consider updating goal coordinate


        # Compute waypoints that approximate circle
        circle_points = self.compute_circle_waypoints(tag_latitude, tag_longitude, 4, 8) #2nd and third parameters are radius (in m) and increments (number of waypoints)

        # This needs testing/work
        for point in circle_points.reversed():

            core.waypoint_handler.waypoints.appendleft(("POSITION", point))
        
        self.logger.info(f"Added position waypoints that go in circle around AR tag: lat ({tag_latitude}), lon ({tag_longitude})")


        # Turn 90 degrees right ---- ***** THIS IS PROBABLY UNNECCESSARY ***** #
        # The rover should turn when its navigating to the provided waypoints around the tag
        speed = 50 #Maybe change this?
        speed1, speed2 = interfaces.driveboard.calculate_move(speed, 90)
        drive_board.send_drive(speed1, speed2)




        # On second tag detection
        if core.vision.ar_tag_detector.is_gate():
            
            interfaces.drive_board.stop()

            # Sleep for a brief second
            await asyncio.sleep(0.1)

            self.logger.info("Gate Search: Gate seen")

            return self.on_event(core.AutonomyEvents.GATE_SEEN)
        

        # We need to somehow enter the approaching marker state if the rover does not find another tag.


        return self




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
# 4. 
#
#
#
