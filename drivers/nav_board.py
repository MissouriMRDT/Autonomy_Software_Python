from drivers.rovecomm import RoveCommEthernetUdp
import constants
import time
import datetime
from drivers.logging import LogWriter

NAV_IP_ADDRESS = "136"
GPS_DATA_ID = 5100
IMU_DATA_ID = 5101
LIDAR_DATA_ID = 5102
GPSADD_DATA_ID = 5103


class NavBoard:
    def __init__(self, rove_comm, logger):
        self._pitch = 0
        self._roll = 0
        self._heading = 0
        self._location = constants.Coordinate(0, 0)
        self._distToGround = 0
        self._lidarQuality = 0 # int 5 for brand new data, counts down 1 every 50ms, should never go below 3.
        self._lastTime = time.time()
        self.rove_comm_node = rove_comm

        self.rove_comm_node.subscribe(NAV_IP_ADDRESS)

        self.rove_comm_node.callbacks[IMU_DATA_ID] = self.process_imu_data
        self.rove_comm_node.callbacks[GPS_DATA_ID] = self.process_gps_data
        self.rove_comm_node.callbacks[LIDAR_DATA_ID] = self.process_lidar_data
        self.logger = logger # see CannyTracking to add logging

    def process_imu_data(self, packet):
        self._pitch, self._heading, self._roll = packet.data
        """        
        self._pitch = packet.data[0] #leave these alone for now, early testing will be hurt by them right now
        if abs(self._heading - packet.data[1]) < 30: # attempt to filter inappropriate values, experimentally i've never seen the rover turn more than 120 degrees per second.
            self._heading = packet.data[1]
        self._roll = packet.data[2] #leave these alone for now, early testing will be hurt by them right now
        """

    def process_gps_data(self, packet):
        # The GPS sends data as two int32_t's
        lon, lat = packet.data
        lat = lat * 1e-7
        lon = -lon * 1e-7
        timeDifference = time.time() - self._lastTime
        # print("5")
        # print(timeDiffernce)
        # print(time.localtime(time.time()))
        print(str(time.time()))
        print(self._location)
        self._lastTime = time.time()
        self._location = constants.Coordinate(lat, lon)

    def process_lidar_data(self, packet):
        self._distToGround, self._lidarQuality = packet.data # LiDAR still needs to be implemented on NavBoard, don't use it on Autonomy

    def pitch(self):
        return self._pitch

    def roll(self):
        return self._roll

    def heading(self):
        return self._heading

    def location(self):
        return self._location


if __name__ == '__main__':
    rove_comm_node = RoveCommEthernetUdp()
    nav = NavBoard(rove_comm_node)
    while True:
        print(nav.location())
        print(nav.heading())
        print(nav.pitch())
        print(nav.roll())
        print("...")
        time.sleep(1)
