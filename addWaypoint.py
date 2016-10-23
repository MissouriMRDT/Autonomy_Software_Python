from gps import GPS
import pickle

gps = GPS("/dev/ttyS0")
waypoints = []

choice = raw_input("a to add a waypoint, q to quit\n")
while choice == "a":
	waypoints.append(gps.location())
	print("Waypoints: ", waypoints)
	choice = raw_input("a to add another waypoint, q to quit")

with open('waypoints.dat','wb') as waypointfile:
	pickle.dump(waypoints, waypointfile, pickle.HIGHEST_PROTOCOL)
	
print "Waypoints saved"