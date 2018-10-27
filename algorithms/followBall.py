import logging
import constants


def drive_to_marker(drive_board, tracker, center, radius):
    angle_to_ball = constants.FIELD_OF_VIEW * ((center[0] - constants.WIDTH / 2) / constants.WIDTH)
    distance = constants.SCALING_FACTOR / radius
    logging.info("Distance to marker: %f" % distance)

    if distance > constants.TARGET_DISTANCE:
        logging.info("Angle to marker: %f" % angle_to_ball)
        return drive_board.calculate_move(constants.DRIVE_POWER, angle_to_ball), distance

    if distance <= constants.TARGET_DISTANCE:
        return drive_board.calculate_move(-constants.DRIVE_POWER, angle_to_ball), distance
