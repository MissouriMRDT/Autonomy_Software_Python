# AUTHORS: COOL PEOPLE (aka Donovan Bale, Aaron Dobsch, and Duncan Truitt)

import asyncio
from algorithms import geomath
from core.vision import obstacle_avoidance
from core.waypoints import WaypointHandler
import core
import interfaces
import algorithms



from core.states import RoverState


class GateSearch(RoverState):

    # Drive to 4m of marker.

    current = interfaces.nav_board.location()
    goal, start, leg_type = gps_data.data()

    bearing, distance = geomath.haversine(current[0],current[1],goal[0],goal[1])

    goal_latitude, goal_longitude = geomath.reverse_haversine(bearing, distance-4, current[0], current[1])
    
    waypoint = core.Coordinate(goal_latitude, goal_longitude)
    core.waypoint_handler.waypoints.appendleft(("GATE", waypoint))

    #self.logger.info(f"Added Position Waypoint to Front of Queue: lat ({goal_latitude}), lon ({goal_longitude})")
    

    # Turn 90 degrees right

    speed1, speed2 = interfaces.driveboard.calculate_move(speed, 90)
    drive_board.send_drive(speed1, speed2)
    #asyncio.sleep(5) --- Maybe?


    # Drive left in a 2.5 meter radius circle

    


    # If marker found within the distance of the circumfrance, enter approaching gate state.
    #
    # else, handle marker not found case.