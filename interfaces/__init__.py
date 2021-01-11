from interfaces.drive_board import DriveBoard
from interfaces.nav_board import NavBoard

# Create instances of all interfaces
drive_board: DriveBoard
nav_board: NavBoard


def setup():
    """
    Sets up all of the interfaces
    """
    global drive_board, nav_board
    drive_board = DriveBoard()
    nav_board = NavBoard()
