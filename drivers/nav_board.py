from drivers.rovecomm import RoveComm
import struct
import time

NAV_IP_ADDRESS = '192.168.1.133'
ACC_DATA_ID = 1314
GYR_DATA_ID = 1315
MAG_DATA_ID = 1316
GPS_DATA_ID = 1297


class NavBoard:
    def __init__(self, rove_comm):
        self._mag = (0, 0, 0)
        self._acc = (0, 0, 0)
        self._gyr = (0, 0, 0)
        self._location = (0, 0)

        self.rove_comm_node = rove_comm

        self.rove_comm_node.subscribe(NAV_IP_ADDRESS)

        self.rove_comm_node.callbacks[MAG_DATA_ID] = self.process_mag_data
        self.rove_comm_node.callbacks[ACC_DATA_ID] = self.process_acc_data
        self.rove_comm_node.callbacks[GYR_DATA_ID] = self.process_gyr_data
        self.rove_comm_node.callbacks[GPS_DATA_ID] = self.process_gps_data

    def process_mag_data(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)
        self._mag = (x, y, z)

    def process_acc_data(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)
        self._acc = (x, y, z)

    def process_gyr_data(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)
        self._gyr = (x, y, z)

    def process_gps_data(self, raw_data):
        # The GPS sends data as two doubles
        lon, lat = struct.unpack("<ll", raw_data)
        lat = lat * 1e-7
        lon = -lon * 1e-7
        self._location = (lat, lon)

    def magnetometer_xyz(self):
        return self._mag

    def accelerometer_xyz(self):
        return self._acc

    def gyroscope_xyz(self):
        return self._gyr

    def location(self):
        return self._location


if __name__ == '__main__':
    rove_comm_node = RoveComm()
    nav = NavBoard(rove_comm_node)
    while True:
        print(nav.location())
        print(nav.magnetometer_xyz())
        print(nav.accelerometer_xyz())
        print(nav.gyroscope_xyz())
        print("...")
        time.sleep(1)
