#
# Mars Rover Design Team
# nav_board.py
#
# Created on Jul 19, 2020
# Updated on Aug 21, 2022
#

import core
from core.constants import Coordinate
import time
import logging
import utm


class NavBoard:
    """
    Interface for the navboard, a separate compute unit that provides GPS and IMU
    (Pitch/Yaw/Roll) data to the Autonomy system. This interface collects and stores the
    data received from the navboard, so it can be used elsewhere.
    """

    def __init__(self):
        self._pitch: float = 0
        self._roll: float = 0
        self._heading: float = 0
        self._location: Coordinate = Coordinate(0, 0)
        self._distToGround: int = 0
        self._lidarQuality = 0  # int 5 for brand-new data, counts down 1 every 50ms, should never go below 3.
        self._lastTime = time.time()
        self._start_UTM = None

        # Set up RoveComm and Logger
        self.logger = logging.getLogger(__name__)

        core.rovecomm_node.udp_node.subscribe(core.manifest["Nav"]["Ip"])

        # set up appropriate callbacks so we can store data as we receive it from NavBoard
        core.rovecomm_node.set_callback(core.manifest["Nav"]["Telemetry"]["IMUData"]["dataId"], self.process_imu_data)
        core.rovecomm_node.set_callback(core.manifest["Nav"]["Telemetry"]["GPSLatLon"]["dataId"], self.process_gps_data)

    def process_imu_data(self, packet):
        """
        Process IMU Data
        :param packet: pitch, heading, and roll included
        """

        self._pitch, self._heading, self._roll = packet.data
        self.logger.debug(f"Incoming IMU data: ({self._pitch}, {self._heading}, {self._roll})")

    def process_gps_data(self, packet) -> None:
        """
        Process GPS Data
        :param packet: lat and lon included
        """

        # The GPS sends data as two int32_t's
        lat, lon = packet.data
        self.logger.debug(f"Incoming GPS data: ({lat}, {lon})")
        self._lastTime = time.time()
        self._location = Coordinate(lat, lon)

    def pitch(self) -> float:
        return self._pitch

    def roll(self) -> float:
        return self._roll

    def heading(self) -> float:
        # Check if ZED relative positioning is turned on.
        if self.
        heading = core.vision.camera_handler.get_pose()[4]
        if heading < 0:
            heading = 360 + heading
        return heading

    def location(self) -> Coordinate:
        location = core.vision.camera_handler.get_pose()
        
        # Get zed x, y location.
        x, y = location[0] / 1000, location[2] / 1000
        
        # Check if we already set are absolute start position.
        if self._start_UTM is None:
            # Get current GPS.
            self._start_UTM =  utm.from_latlon(self._location[0], self._location[1])

        # Add Start UTM coords to ZED position.
        x, y = x + self._start_UTM[0], y + self._start_UTM[1]
        # Convert back to GPS. Last two params are UTM zone.
        gps_current = utm.to_latlon(*(x, y, self._start_UTM[2], self._start_UTM[3]))
        return Coordinate(gps_current[0], gps_current[1])
