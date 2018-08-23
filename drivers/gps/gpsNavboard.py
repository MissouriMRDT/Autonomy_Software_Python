from drivers.rovecomm import RoveComm
from collections import namedtuple
import struct
import time

# GPS class
# 3/4/2017
# written by Ashley Painter

Coordinate = namedtuple('Coordinate', ['lat', 'lon'])

# the ip address and data id of the gps
# ip address may not be correct
gps_ip_address = '192.168.1.133'
gps_data_id = 1297


class GPS:
    def __init__(self, rovecomm):
        self._location = (0, 0)
        self.location_history = []
        self.rovecomm_node = rovecomm
        # subscribe to device
        self.rovecomm_node.subscribe(gps_ip_address)
        # Tell rovecomm where to put the data when it comes in
        self.rovecomm_node.callbacks[gps_data_id] = self.process_gps_data

    # I'm not sure if this will cuase an error
    def process_gps_data(self, raw_data):
        # The GPS sends data as two doubles
        lon, lat = struct.unpack("<ll", raw_data)
        lat = lat * 1e-7
        lon = -lon * 1e-7
        self._location=(lat, lon)

    def location(self):
        # returns most recent reported location
        return Coordinate(*self._location)

if __name__ == '__main__':
    rovecomm_node = RoveComm()
    gps = GPS(rovecomm_node)
    while(True):
        print(gps.location())
        time.sleep(1)
