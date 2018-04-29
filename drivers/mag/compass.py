from drivers.rovecomm import RoveComm
import struct
import time
import json
import math
import logging

# from numpy import interp

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


def interp(x, inrange, outrange):
    inspan = inrange[1] - inrange[0]
    outspan = outrange[1] - outrange[0]
    scaled = (x - inrange[0]) / inspan
    return (scaled * outspan) + outrange[0]


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
        try:
            with open(calibration_file) as calfile:
                self._calibration = json.load(calfile)
                self.x_range = [self._calibration['min_x'], self._calibration['max_x']]
                self.y_range = [self._calibration['min_y'], self._calibration['max_y']]
                self.offset = self._calibration['due_north_offset']
        except IOError:
            logging.critical("No calibration data available")
        except ValueError:
            logging.critical("Error: Calibration data corrupted. Try recalibrating")

    def process_mag_data(self, raw_data):
        x, y, z = struct.unpack("fff", raw_data)

        # Apply the moving average filter to reduce noise in the data
        x = self._filter_x.update(x)
        y = self._filter_y.update(y)
        z = self._filter_z.update(z)
        self._coordinates = (x, y, z)
        cal = self._calibration
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


if __name__ == '__main__':
    """
    Running the magnetometer standalone will run the calibration program
    """

    import thread
    import numpy

    rovecomm_node = RoveComm()


    def do_nothing(packet_contents):
        pass

        ignored_data_ids = [1313, 1314, 1315, 1296, 1297, 1298, 1299, 1300, 1301, 1313, 1314, 1315]
        for id in ignored_data_ids:
            rovecomm_node.callbacks[id] = do_nothing


    mag = Compass(rovecomm_node)
    x_meas, y_meas, z_meas = [], [], []
    done_measuring = False


    def sample_magnetometer(x_meas, y_meas):
        while not done_measuring:
            reading = mag.raw_xyz()
            print("Measured %s" % (reading,))
            x_meas.append(reading[0])
            z_meas.append(reading[2])
            y_meas.append(reading[1])

            time.sleep(0.3)


    choice = raw_input("Calibrate the magnetometer (y/n)?")
    if (choice == 'y'):
        print(
            "Press enter to start calibration. Press enter again when facing due north after turning at least 360 degrees")
        raw_input()
        thread.start_new_thread(sample_magnetometer, (x_meas, y_meas))
        raw_input()
        done_measuring = True

        cal = {"min_x": min(x_meas), "max_x": max(x_meas), "min_y": min(y_meas), "max_y": max(y_meas)}
        (x, y, z) = mag.raw_xyz()
        x_adj = numpy.interp(x, [cal["min_x"], cal["max_x"]], [-1, 1])
        y_adj = numpy.interp(y, [cal["min_y"], cal["max_y"]], [-1, 1])
        heading = math.degrees(math.atan2(y_adj, x_adj))
        heading = heading + 360 if heading < 0 else heading
        cal['due_north_offset'] = heading
        print("Due north offset: %f" % heading)
        with open('mag_calibration.json', 'w') as calfile:
            json.dump(cal, calfile)

        print("Calibration completed.")
        print("Press enter to display heading, ctrl-c to exit\n")
        input()

    mag = Compass(rovecomm_node)

    while (True):
        print("Heading: %.02f " % mag.heading())
        time.sleep(0.2)
