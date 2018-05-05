import time

from drivers.gps.gpsNavboard import GPS
from drivers.rovecomm import RoveComm
from algorithms.geoMath import Coordinate

rovecomm_node = RoveComm()
gps = GPS(rovecomm_node)

while True:
    location = gps.location()
    print("\tLocation: %s, %s" %(location.lat, location.lon))
    print("...")
    time.sleep(5)
