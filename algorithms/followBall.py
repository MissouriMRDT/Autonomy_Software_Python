import logging
import constants


def drive_to_marker(speed, drive_board, center, radius):
    angle_to_ball = constants.FIELD_OF_VIEW * ((center[0] - constants.WIDTH / 2) / constants.WIDTH)
    print(angle_to_ball)
    distance = constants.SCALING_FACTOR / radius
    logging.info("Distance to marker: %f" % distance)
    logging.info("Angle to marker: %f" % angle_to_ball)
    return drive_board.calculate_move(speed, angle_to_ball), distance
