
def get_motor_power_from_heading(goal_heading, drive_board, nav_board):
    if abs(goal_heading - nav_board.heading()) > 5:
        return drive_board.calculate_move(50, goal_heading - nav_board.heading())
    return 50, 50
