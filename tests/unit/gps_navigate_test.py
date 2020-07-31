#yeah... we are just importing everything for now
import core.rover_states as rs
import core.constants as constants
from core.constants import ApproachState
from core.rovecomm import RoveCommEthernetUdp
from interfaces.drive_board import DriveBoard
from interfaces.nav_board import NavBoard
import algorithms.gps_navigate as gps_nav
import algorithms.marker_search as marker_search
from algorithms.gps_navigate import GPSData
import algorithms.geomath as geomath

#Rolla GPS coordinates
rolla_coord = constants.Coordinate(37.951424, -91.768959)

def test_get_approach_status_past_goal():
    #set up goal to be due north of Rolla coordinates, set up current location to be further north
    goal_coord = constants.Coordinate(37.951524, -91.768959)
    current_coord = constants.Coordinate(37.951724, -91.768959)
    
    #this will trigger a past goal warning, as our target bearing has now effectively flipped
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.PAST_GOAL

    #set up goal to be due north of Rolla coordinates, set up current location to be south of start
    goal_coord = constants.Coordinate(37.951524, -91.768959)
    current_coord = constants.Coordinate(37.951224, -91.768959)
    
    #this will should not trigger a past goal warning, as our bearing should be very similar
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.ON_COURSE


def test_get_approach_status_close_enough():
    #set up our current position and target position to be within 0.1m
    #this is a rough approximation disregarding the haversine math but it should serve

    goal_coord = constants.Coordinate(37.951824, -91.768959)
    current_coord = constants.Coordinate(goal_coord.lat-0.000001, goal_coord.lon)
    
    #this should trigger a CLOSE_ENOUGH state as we are within a tiny threshold
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.CLOSE_ENOUGH

    #set up our current position and target position to be more than 10m

    goal_coord = constants.Coordinate(37.951824, -91.768959)
    current_coord = constants.Coordinate(goal_coord.lat-0.0001, goal_coord.lon)
    
    #this should trigger a ON_COURSE state as we are a decent ways away
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.ON_COURSE







