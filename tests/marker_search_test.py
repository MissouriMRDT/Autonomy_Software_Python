import logging
import time
import constants
import rover_states
import algorithms.marker_search as marker_search
import algorithms.gps_navigate as gps_nav

state_switcher = rover_states.StateSwitcher()
state_switcher.state = rover_states.Searching()
gps_data = gps_nav.GPSData()

gps_data.start = constants.Coordinate(39, 111)
current_loc = gps_data.start
gps_data.goal = marker_search.calculate_next_coordinate(gps_data.start, current_loc)
print("Origin: " + str(gps_data.start))

while True:

    if state_switcher.state == rover_states.Searching():

        print()
        goal, start = gps_data.data()

        print("Goal: " + str(goal))

        new_goal = marker_search.calculate_next_coordinate(start, current_loc)

        current_loc = goal
        print(current_loc)
        gps_data.goal = new_goal

    time.sleep(2)
