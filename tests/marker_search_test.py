import logging
import time
import constants
import rover_states
import algorithms.marker_search as marker_search
import algorithms.gps_navigate as gps_nav

state_switcher = rover_states.StateSwitcher()
state_switcher.state = rover_states.Searching()
gps_data = gps_nav.GPSData()

gps_data.start = constants.Coordinate(37.950271, -91.777770)
current_loc = gps_data.start
gps_data.goal = marker_search.calculate_next_coordinate(gps_data.start, current_loc)
print("Origin: " + str(gps_data.start.lat) + ", " + str(gps_data.start.lon))

while True:

    if state_switcher.state == rover_states.Searching():

        print()
        goal, start = gps_data.data()

        #print("Current: " + str(current_loc))
        #print("Goal: " + str(goal))

        new_goal = marker_search.calculate_next_coordinate(start, current_loc)

        print("New Goal: " + str(new_goal.lat) + ", " + str(new_goal.lon))
        current_loc = goal
        gps_data.goal = new_goal

    time.sleep(1)
