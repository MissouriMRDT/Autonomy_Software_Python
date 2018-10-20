
def get_motor_power_from_heading(goal_heading, drive_board, nav_board):
    if abs(goal_heading - nav_board.heading()) < 5 or abs(nav_board.roll()) > 15:
        return drive_board.calculate_move(10, goal_heading - nav_board.heading())
    return 10, 10
