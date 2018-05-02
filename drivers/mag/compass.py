from drivers.rovecomm import RoveComm
import struct
import json
import math
import logging

from numpy import interp

MAG_IP_ADDRESS = '192.168.1.133'
MAG_DATA_ID = 1316
FILTER_COEFFICIENT = 0.25


class AveragingLowpassFilter(object):
    def __init__(self, coefficient, init_val=0):
        """
        coefficient: Multiplier for new data. 
                     Higher number = faster reaction. 
                     0 < coefficient < 1
        """
        self.c_new = coefficient
        self.c_old = 1 - coefficient
        self.val = init_val

    def update(self, raw_data):
        self.val = (self.val * self.c_old) + raw_data * self.c_new
        return self.val


class Compass:
    def __init__(self, rovecomm, calibration_file="mag_calibration.json"):
        self._coordinates = (0, 0, 0)

        self.rovecomm_node = rovecomm
        # subscribe to device
        self.rovecomm_node.subscribe(MAG_IP_ADDRESS)
        # Tell rovecomm where to put the data when it comes in
        self.rovecomm_node.callbacks[MAG_DATA_ID] = self.process_mag_data
        self._heading = 0

        # Initialize filters
        self._filter_x = AveragingLowpassFilter(FILTER_COEFFICIENT)
        self._filter_y = AveragingLowpassFilter(FILTER_COEFFICIENT)
        self._filter_z = AveragingLowpassFilter(FILTER_COEFFICIENT)

        #Read in the calibration for the compass
        try:
            with open(calibration_file) as calfile:
                self._calibration = json.load(calfile)
                self.x_range = [self._calibration['min_x'], self._calibration['max_x']]
                self.y_range = [self._calibration['min_y'], self._calibration['max_y']]
                self.offset = self._calibration['due_north_offset']
        except IOError:
            logging.critical("No calibration data available!")
        except ValueError:
            logging.critical("Error: Calibration data corrupted. Try recalibrating")

    def process_mag_data(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)

        # Apply the moving average filter to reduce noise in the data
        x = self._filter_x.update(x)
        y = self._filter_y.update(y)
        z = self._filter_z.update(z)
        self._coordinates = (x, y, z)

        x_adj = interp(x, self.x_range, [-1, 1])
        y_adj = interp(y, self.y_range, [-1, 1])
        heading = math.degrees(math.atan2(y_adj, x_adj))
        heading = (heading - self.offset) % 360
        # logging.info("Data received. Raw = %s, Heading = %f" % ([x_adj, y_adj], heading))
        self._heading = heading

    def heading(self):
        return self._heading

    def raw_xyz(self):
        return self._coordinates