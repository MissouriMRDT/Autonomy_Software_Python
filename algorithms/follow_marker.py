import logging
import core
import algorithms.heading_hold as hh  # hh to match other styles in our code base


def drive_to_marker(speed, drive_board, center, radius, nav_board):
    angle_to_ball = core.FIELD_OF_VIEW * ((center[0] - core.WIDTH / 2) / core.WIDTH)
    print(angle_to_ball)
    if radius == 0:
        radius = 1  # TODO: more robust fix
    distance = core.SCALING_FACTOR / radius
    logging.info("Distance to marker: %f" % distance)
    logging.info("Angle to marker: %f" % angle_to_ball)
    goal_heading = nav_board._heading + angle_to_ball
    left, right = hh.get_motor_power_from_heading(speed, goal_heading, drive_board, nav_board)
    print(str(left) + "," + str(right))
    return (left, right), distance
