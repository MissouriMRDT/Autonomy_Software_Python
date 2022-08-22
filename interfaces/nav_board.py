import core
import time
import logging


class NavBoard:
    """
    Interface for the navboard, a seperate compute unit that provides GPS and IMU
    (Pitch/Yaw/Roll) data to the Autonomy system. This interface collects and stores the
    data received from the navboard so it can be used elsewhere.
    """

    def __init__(self):
        self._pitch: float = 0
        self._roll: float = 0
        self._heading: float = 0
        self._location: core.Coordinate = core.Coordinate(0, 0)
        self._distToGround: int = 0
        self._lidarQuality = 0  # int 5 for brand new data, counts down 1 every 50ms, should never go below 3.
        self._lastTime = time.time()

        # Set up RoveComm and Logger
        self.logger = logging.getLogger(__name__)

        core.rovecomm_node.udp_node.subscribe(core.manifest["Nav"]["Ip"])

        # set up appropriate callbacks so we can store data as we receive it from NavBoard
        core.rovecomm_node.set_callback(core.manifest["Nav"]["Telemetry"]["IMUData"]["dataId"], self.process_imu_data)
        core.rovecomm_node.set_callback(core.manifest["Nav"]["Telemetry"]["GPSLatLon"]["dataId"], self.process_gps_data)

    def process_imu_data(self, packet):
        self._pitch, self._heading, self._roll = packet.data
        self.logger.debug(f"Incoming IMU data: ({self._pitch}, {self._heading}, {self._roll})")

    def process_gps_data(self, packet) -> None:
        # The GPS sends data as two int32_t's
        lat, lon = packet.data
        self.logger.debug(f"Incoming GPS data: ({lat}, {lon})")
        self._lastTime = time.time()
        self._location = core.Coordinate(lat, lon)

    def pitch(self) -> float:
        return self._pitch

    def roll(self) -> float:
        return self._roll

    def heading(self) -> float:
        return self._heading

    def location(self) -> core.Coordinate:
        return self._location
