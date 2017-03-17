from rovecomm import RoveComm
import struct
import time

MAG_IP_ADDRESS = '192.168.1.133'
MAG_DATA_ID = 1316


class Compass:
    def __init__(self, rovecomm):
        self._coordinates = (None, None, None)
        self.rovecomm_node = rovecomm
        # subscribe to device
        self.rovecomm_node.subscribe(MAG_IP_ADDRESS)
        # Tell rovecomm where to put the data when it comes in
        self.rovecomm_node.callbacks[MAG_DATA_ID] = self.process_mag_data

    # I'm not sure if this will cause an error
    def process_mag_data(self, raw_data):
        # The GPS sends data as two doubles
        x, y, z = struct.unpack("fff", raw_data)
        self._coordinates = (x, y, z)

    def heading(self):
        # returns most recent reported raw mag data
        return self._coordinates


if __name__ == '__main__':
    rovecomm_node = RoveComm()
    mag = Compass(rovecomm_node)
    while(True):
        print mag.heading()
        time.sleep(1)