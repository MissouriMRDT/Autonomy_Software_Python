import core.constants as constants
from core.constants import ApproachState
from core.rovecomm import RoveCommEthernetUdp
from interfaces.drive_board import DriveBoard
from interfaces.nav_board import NavBoard
import algorithms.gps_navigate as gps_nav
import time
from core.logging import LogWriter

'''
UNIT TEST
FILE: gps_navigate.py

This file provides unit tests for both the get_approach_status and calculate_move() 
'''

# Rolla GPS coordinates
rolla_coord = constants.Coordinate(37.951424, -91.768959)

# set up dependancies
outString = time.strftime("%Y%m%d-%H%M%S") + ".txt"
Logger = LogWriter(outString)
rovecomm_node = RoveCommEthernetUdp(Logger)

# set up interfaces
drive_board = DriveBoard()
nav_board = NavBoard(rovecomm_node, Logger)


def test_get_approach_status_past_goal():
    # set up goal to be due north of Rolla coordinates, set up current location to be further north
    goal_coord = constants.Coordinate(rolla_coord.lat + 0.001, rolla_coord.lon)
    current_coord = constants.Coordinate(rolla_coord.lat + 0.003, rolla_coord.lon)

    # this will trigger a past goal warning, as our target bearing has now effectively flipped
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.PAST_GOAL


def test_get_approach_status_close_enough():
    # set up our current position and target position to be within 0.1m
    # this is a rough approximation disregarding the haversine math but it should serve

    goal_coord = constants.Coordinate(37.951824, -91.768959)
    current_coord = constants.Coordinate(goal_coord.lat - 0.000001, goal_coord.lon)

    # this should trigger a CLOSE_ENOUGH state as we are within a tiny threshold
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.CLOSE_ENOUGH


def test_get_approach_status_approaching():
    # set up our current position and target position to be more than 10m apart
    goal_coord = constants.Coordinate(rolla_coord.lat + 0.0003, rolla_coord.lon)
    current_coord = constants.Coordinate(goal_coord.lat - 0.0001, goal_coord.lon)

    # this should trigger a APPROACHING state as we are a decent ways away
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.APPROACHING

    # set up goal to be due north of Rolla coordinates, set up current location to be south of start
    goal_coord = constants.Coordinate(37.951524, -91.768959)
    current_coord = constants.Coordinate(37.951224, -91.768959)

    # this will should not trigger a past goal warning, we haven't past the goal yet and are not close enough
    assert gps_nav.get_approach_status(goal_coord, current_coord, rolla_coord) == ApproachState.APPROACHING


def test_calculate_move_right():
    # set up our goal position to be east of our current
    # heading is 0 (we are pointing straight north)
    goal_coord = constants.Coordinate(rolla_coord.lat, rolla_coord.lon + 0.005)
    current_coord = constants.Coordinate(rolla_coord.lat, rolla_coord.lon + 0.0025)

    drive_board.enable()
    left, right = gps_nav.calculate_move(goal_coord, current_coord, rolla_coord, drive_board, nav_board)

    # should be turning to the right
    assert right < 0
    assert left > 0


def test_calculate_move_left():
    # set up our goal position to be west of our current
    # heading is 0 (we are pointing straight north)
    goal_coord = constants.Coordinate(rolla_coord.lat, rolla_coord.lon - 0.005)
    current_coord = constants.Coordinate(rolla_coord.lat, rolla_coord.lon - 0.0025)

    drive_board.enable()
    left, right = gps_nav.calculate_move(goal_coord, current_coord, rolla_coord, drive_board, nav_board)

    # should be turning to the left
    assert right > 0
    assert left < 0


def test_calculate_move_straight():
    # set up our goal position to be north of our current
    # heading is 0 (we are pointing straight north)
    goal_coord = constants.Coordinate(rolla_coord.lat + 0.0004, rolla_coord.lon)
    current_coord = constants.Coordinate(rolla_coord.lat + 0.0002, rolla_coord.lon)

    drive_board.enable()
    left, right = gps_nav.calculate_move(goal_coord, current_coord, rolla_coord, drive_board, nav_board)

    # should not have to turn
    assert right == left
