from interfaces.drive_board import DriveBoard
from interfaces.nav_board import NavBoard
from interfaces.multimedia_board import MultiMedia
import sys

# Create instances of all interfaces
drive_board: DriveBoard

nav_board: NavBoard

multimedia_board: MultiMedia


def setup():
    """
    Sets up all of the interfaces
    """
    global drive_board, nav_board, multimedia_board
    nav_board = NavBoard()
    drive_board = DriveBoard()
    multimedia_board = MultiMedia()
