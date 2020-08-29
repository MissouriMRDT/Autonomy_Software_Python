import turtle
import algorithms.marker_search as marker_search
import core.constants as constants

'''
UNIT TEST
FILE: marker_search.py

DESCRIPTION: This file provides a unit test for the search pattern generation. It compares the output of a turtle (python) drawing
to that of a previously correctly generated one. 

PROCEDURES: If any changes are made, place this file in the root directory and run it directly to confirm and update the expected output.
'''


def compareOutputs(output, expected):
    '''
    Compare the two outputs to see if the marker_search
    code still outputs the same pattern, we compare line by
    line as these are PS files
    '''

    with open(output, 'r') as file1:
        with open(expected, 'r') as file2:
            diff = set(file1).difference(file2)

    # the only difference between the files should be the creation date
    if len(diff) == 1 and 'CreationDate' in next(iter(diff)):
        for line in diff:
            print(line)
        return True

    else:
        for line in diff:
            print(line)
    return False


def drawLogarithmicSpiral(r=50, fileName="resources/tests/output_search_pattern.eps"):
    '''
    Draws and outputs the pattern provided by marker_search using the turtle module
    '''
    t = turtle.Turtle()
    t.up()

    x = 0
    y = 0

    # pick the same GPS coord for starting and curren pos
    start_coord = constants.Coordinate(37.951424, -91.768959)
    end_coord = constants.Coordinate(37.951424, -91.768959)

    t.setpos(x, y)
    t.down()
    t.hideturtle()
    t.speed(0)

    for i in range(r):
        # generate the next point in spiral
        new_coord = marker_search.calculate_next_coordinate(start_coord, end_coord)

        # calculate the deltas in position, we can't represent the precision of GPS coords in turtle
        dx = new_coord.lon - end_coord.lon
        dy = new_coord.lat - end_coord.lat

        # scale deltas to make visible on screen
        x += dx * 500000
        y += dy * 500000

        t.setpos(x, y)
        end_coord = constants.Coordinate(new_coord.lat, new_coord.lon)

    # now export the turtle drawing to the resources folder
    ts = t.getscreen()
    ts.getcanvas().postscript(file=fileName)


def test_calculate_next_coordinate():
    '''
    Compare the generated marker search with the expected, assert that they are the same
    '''

    drawLogarithmicSpiral()
    assert compareOutputs('resources/tests/output_search_pattern.eps', 'resources/tests/expected_search_pattern.eps') == True


if __name__ == "__main__":
    drawLogarithmicSpiral(fileName="resources/tests/expected_search_pattern.eps")
    turtle.mainloop()