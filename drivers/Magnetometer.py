from rovecomm import RoveComm
import struct
import time
import json
import math

MAG_IP_ADDRESS = '192.168.1.133'
MAG_DATA_ID = 1316


class Compass:
    def __init__(self, rovecomm,  calibration_file="mag_calibration.json"):
        self._coordinates = (0, 0, 0)
        self.rovecomm_node = rovecomm
        # subscribe to device
        self.rovecomm_node.subscribe(MAG_IP_ADDRESS)
        # Tell rovecomm where to put the data when it comes in
        self.rovecomm_node.callbacks[MAG_DATA_ID] = self.process_mag_data
        try:
            with open(calibration_file) as calfile:
                self._calibration = json.load(calfile)
        except IOError:
            print "Error: No calibration data available"
        except ValueError:
            print "Error: Calibration data corrupted. Try recalibrating"


    # I'm not sure if this will cause an error
    def process_mag_data(self, raw_data):
        # The GPS sends data as two doubles
        x, y, z = struct.unpack("fff", raw_data)
        self._coordinates = (x, y, z)

    def heading(self):
        x, y, z = self._coordinates
        cal = self._calibration
        x_adj = (x - cal['min_x']) / (cal['max_x'] - cal['min_x'])
        y_adj = (y - cal['min_y']) / (cal['max_y'] - cal['min_y'])
        heading = math.atan(x_adj / y_adj)
        return heading

    def raw_xyz(self):
        return self._coordinates


if __name__ == '__main__':
    """
    Running the magnetometer standalone will run the calibration program
    """

    import thread
    rovecomm_node = RoveComm()
    def do_nothing(packet_contents):
        pass
    rovecomm_node.callbacks[1313] = do_nothing
    rovecomm_node.callbacks[1314] = do_nothing
    rovecomm_node.callbacks[1315] = do_nothing
    rovecomm_node.callbacks[1296] = do_nothing
    mag = Compass(rovecomm_node)
    x_meas, y_meas = [], []
    done_measuring = False


    def sample_magnetometer(x_meas, y_meas):
        while not done_measuring:
            reading = mag.raw_xyz()
            print("Measured %s" % (reading,))
            x_meas.append(reading[0])
            y_meas.append(reading[1])
            time.sleep(0.3)


    print("Press enter to start calibration. Press enter again when facing due north after turning at least 360 degrees")
    raw_input()
    thread.start_new_thread(sample_magnetometer, (x_meas, y_meas))
    raw_input()
    done_measuring = True

    cal = {"min_x": min(x_meas), "max_x": max(x_meas), "min_y": min(y_meas), "max_y": max(y_meas)}
    (x, y, z) = mag.raw_xyz()
    x_adj = (x - cal['min_x']) / (cal['max_x'] - cal['min_x'])
    y_adj = (y - cal['min_y']) / (cal['max_y'] - cal['min_y'])
    heading = math.atan(x_adj / y_adj)
    cal['due_north_offset'] = heading
    print("Due north offset: %f" % heading)
    with open('mag_calibration.json', 'w') as calfile:
        json.dump(cal, calfile)

    print("Calibration completed.")
    print("Press enter to display heading, ctrl-c to exit\n")
    raw_input()

    mag = Compass(rovecomm_node)

    while(True):
        print("Heading: %.02f " % mag.heading())
        time.sleep(0.2)