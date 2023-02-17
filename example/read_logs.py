#
# Mars Rover Design Team
# read_logs.py
#
# Created on Nov 05, 2020
# Updated on Aug 21, 2022
#

import logging
import csv
from datetime import datetime
import matplotlib.pyplot as plt


def main() -> None:
    """
    Reads some telemetry from the logs as an example of post-test analysis
    """
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    log_file = "./logs/newformat-20201024-133704.csv"

    # Gets motor differential data
    timestamps = []
    ratios = []
    with open(log_file) as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            if row["MESSAGE"].startswith("Driving at "):
                speeds = eval(row["MESSAGE"][11:])
                ratios.append(speeds[0] - speeds[1])
                timestamps.append(datetime.strptime(row["TIMESTAMP"], "%Y-%m-%d %H:%M:%S.%f"))

    # Graph the data with matplot
    fig, ax = plt.subplots()
    ax.plot(timestamps, ratios)
    plt.title("Motor Differential over Time")
    plt.show()


if __name__ == "__main__":
    # Run main()
    main()
