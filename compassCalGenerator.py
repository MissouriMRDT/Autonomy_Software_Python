import thread
import numpy
import time
import json
import math
from drivers.rovecomm import RoveComm
from drivers.mag.compass import Compass

rovecomm_node = RoveComm()

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
    print("Press enter to start calibration. Press enter again when facing due north after turning at least 360 degrees")
    raw_input()
    thread.start_new_thread(sample_magnetometer, (x_meas, y_meas))
    raw_input()
    done_measuring = True

    cal = {"min_x": min(x_meas), "max_x": max(x_meas),
           "min_y": min(y_meas), "max_y": max(y_meas)}
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
