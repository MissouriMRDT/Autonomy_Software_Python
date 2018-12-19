import constants


def get_motor_power_from_heading(speed, goal_heading, drive_board, nav_board):
    if abs(goal_heading - nav_board.heading()) > 5:
        return drive_board.calculate_move(speed, goal_heading - nav_board.heading())
    return speed, speed
