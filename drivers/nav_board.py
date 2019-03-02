from drivers.rovecomm import RoveCommEthernetUdp
import constants
import time

NAV_IP_ADDRESS = "133"
PITCH_DATA_ID = 1314
ROLL_DATA_ID = 1315
HEADING_DATA_ID = 1316
GPS_DATA_ID = 1297


class NavBoard:
    def __init__(self, rove_comm):
        self._pitch = 0
        self._roll = 0
        self._heading = 0
        self._location = constants.Coordinate(0, 0)

        self.rove_comm_node = rove_comm

        self.rove_comm_node.subscribe(NAV_IP_ADDRESS)

        self.rove_comm_node.callbacks[PITCH_DATA_ID] = self.process_pitch_data
        self.rove_comm_node.callbacks[ROLL_DATA_ID] = self.process_roll_data
        self.rove_comm_node.callbacks[HEADING_DATA_ID] = self.process_heading_data
        self.rove_comm_node.callbacks[GPS_DATA_ID] = self.process_gps_data

    def process_pitch_data(self, packet):
        self._pitch = packet.data[0]

    def process_roll_data(self, packet):
        self._roll = packet.data[0]

    def process_heading_data(self, packet):
        self._heading = packet.data[0]

    def process_gps_data(self, packet):
        # The GPS sends data as two doubles
        lon, lat = packet.data
        lat = lat * 1e-7
        lon = -lon * 1e-7
        self._location = constants.Coordinate(lat, lon)

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
