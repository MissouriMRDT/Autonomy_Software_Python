# from drivers.rovecomm import RoveComm
import struct

NAV_IP_ADDRESS = '192.168.1.133'
ACC_DATA_ID = 1314
GYR_DATA_ID = 1315
MAG_DATA_ID = 1316


class NavBoard:
    def __init__(self, rovecomm):
        self._mag = (0, 0, 0)
        self._acc = (0, 0, 0)
        self._gyr = (0, 0, 0)

        self.rovecomm_node = rovecomm
        # subscribe to device
        self.rovecomm_node.subscribe(NAV_IP_ADDRESS)
        # Tell rovecomm where to put the data when it comes in
        self.rovecomm_node.callbacks[MAG_DATA_ID] = self.processMagData
        self.rovecomm_node.callbacks[ACC_DATA_ID] = self.processAccData
        self.rovecomm_node.callbacks[GYR_DATA_ID] = self.processGyrData

    def processMagData(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)
        self._mag = (x, y, z)

    def processAccData(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)
        self._acc = (x, y, z)

    def processGyrData(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)
        self._gyr = (x, y, z)

    def magnetometerXYZ(self):
        return self._mag

    def accelerometerXYZ(self):
        return self._acc

    def gyroscopeXYZ(self):
        return self._gyr