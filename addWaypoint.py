import json

from drivers.gps_nmea import GPS

gps = GPS("/dev/ttyS0")
waypoints = []

choice = raw_input("a to add a waypoint, q to quit\n")
while choice == "a":
	waypoints.append(gps.location())
	print("Waypoints: ", waypoints)
	choice = raw_input("a to add another waypoint, q to quit")

with open('waypoints.json','wb') as waypointfile:
	json.dump(waypoints, waypointfile)
	
print "Waypoints saved"