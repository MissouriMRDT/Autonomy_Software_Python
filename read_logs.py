import logging
import csv
from datetime import datetime
import matplotlib.pyplot as plt


def main() -> None:
    '''
    Main function for example script, tests geomath code
    '''
    logger = logging.getLogger(__name__)
    logger.info("Executing function: main()")

    # Edit csv file to match the now updated format
    f = open("logs/example-20201024-133704.csv", "r")
    lines = f.read().split('\n')[:-1]
    lines[:] = [x.split(", ") for x in lines]
    lines[:] = [x[:4] + ['"' + ','.join(x[4:]) + '"'] for x in lines]
    f2 = open("logs/newformat-20201024-133704.csv", 'w')
    for line in lines:
        f2.write(','.join(line) + '\n')

    # Graphs left vs right difference over time
    timestamps = []
    ratios = []
    with open("logs/newformat-20201024-133704.csv") as csvfile:
        reader = csv.DictReader(csvfile)
        for row in reader:
            if row['MESSAGE'].startswith('Driving at '):
                speeds = eval(row['MESSAGE'][11:])
                ratios.append(speeds[0] - speeds[1])
                timestamps.append(datetime.strptime(row['TIMESTAMP'], "%Y-%m-%d %H:%M:%S.%f"))

    fig, ax = plt.subplots()
    ax.plot(timestamps, ratios)
    plt.title("Motor Differential over Time")
    plt.show()


if __name__ == "__main__":
    # Run main()
    main()
