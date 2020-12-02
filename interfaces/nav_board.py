import core
import time
import logging


class NavBoard:
    """
    Interface for the navboard, a seperate compute unit that provides GPS and IMU (Pitch/Yaw/Roll) data to the Autonomy system.
    This interface collects and stores the data received from the navboard so it can be used elsewhere.
    """

    def __init__(self):
        self._pitch = 0
        self._roll = 0
        self._heading = 0
        self._location = core.constants.Coordinate(0, 0)
        self._distToGround = 0
        self._lidarQuality = 0  # int 5 for brand new data, counts down 1 every 50ms, should never go below 3.
        self._lastTime = time.time()

        # Set up RoveComm and Logger
        self.logger = logging.getLogger(__name__)

        self.logger = logging.getLogger(__name__)
        core.rovecomm_node.udp_node.subscribe(core.NAV_IP_ADDRESS)

        # set up appropriate callbacks so we can store data as we receive it from NavBoard
        core.rovecomm_node.set_callback(core.IMU_DATA_ID, self.process_imu_data)
        core.rovecomm_node.set_callback(core.GPS_DATA_ID, self.process_gps_data)
        core.rovecomm_node.set_callback(core.LIDAR_DATA_ID, self.process_lidar_data)

    def process_imu_data(self, packet):
        self._pitch, self._heading, self._roll = packet.data

    def process_gps_data(self, packet):
        # The GPS sends data as two int32_t's
        lat, lon = packet.data
        lat = lat * 1e-7
        lon = lon * 1e-7
        self.logger.debug(f"Incoming GPS data: ({lat}, {lon})")
        self._lastTime = time.time()
        self._location = core.constants.Coordinate(lat, lon)

    def process_lidar_data(self, packet):
        self._distToGround, self._lidarQuality = packet.data  # LiDAR still needs to be implemented on NavBoard, don't use it on Autonomy

    def pitch(self):
        return self._pitch

    def roll(self):
        return self._roll

    def heading(self):
        return self._heading

    def location(self):
        return self._location


def main() -> None:
    nav = NavBoard()
    while True:
        print(nav.location())
        print(nav.heading())
        print(nav.pitch())
        print(nav.roll())
        print("...")
        time.sleep(1)


if __name__ == '__main__':
    # Run main
    main()
