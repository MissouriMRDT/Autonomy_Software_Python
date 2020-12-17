import matplotlib.testing.compare as plt_tst
import matplotlib.pyplot as plt
import algorithms.marker_search as marker_search
import core.constants as constants

"""
UNIT TEST
FILE: marker_search.py

DESCRIPTION: This file provides a unit test for the search pattern generation. It compares the output of a matplotlib (python) drawing
to that of a previously correctly generated one. 

PROCEDURES: If any changes are made, place this file in the root directory and run it directly to confirm and update the expected output.
"""


def drawLogarithmicSpiral(r=50, fileName="resources/tests/output/output_search_pattern.png"):
    """
    Draws and outputs the pattern provided by marker_search using the matplotlib module
    """

    x = 0
    y = 0

    # pick the same GPS coord for starting and curren pos
    start_coord = constants.Coordinate(37.951424, -91.768959)
    end_coord = constants.Coordinate(37.951424, -91.768959)

    li = []

    for i in range(r):
        # generate the next point in spiral
        new_coord = marker_search.calculate_next_coordinate(start_coord, end_coord)

        # calculate the deltas in position, we can't represent the precision of GPS coords in matplotlib
        dx = new_coord.lon - end_coord.lon
        dy = new_coord.lat - end_coord.lat

        # update position on screen
        x += dx
        y += dy

        end_coord = constants.Coordinate(new_coord.lat, new_coord.lon)
        li.append((x, y))

    plt.plot(*zip(*li))
    plt.axis("off")
    plt.savefig(fileName)


def test_compare_search_pattern():
    """
    Draw pattern and make sure it is the same as expected
    """

    drawLogarithmicSpiral()
    assert (
        plt_tst.compare_images(
            "resources/tests/expected/expected_search_pattern.png",
            "resources/tests/output/output_search_pattern.png",
            0.001,
        )
        is None
    )


def main():
    drawLogarithmicSpiral(fileName="resources/tests/expected/expected_search_pattern.png")
    plt.show()


if __name__ == "__main__":
    # Call main()
    main()
