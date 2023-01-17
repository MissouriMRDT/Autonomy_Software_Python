from interfaces.drive_board import DriveBoard
from interfaces.nav_board import NavBoard
from interfaces.multimedia_board import MultiMedia
import sys

# reference to self
this = sys.modules[__name__]

# Create instances of all interfaces
drive_board: DriveBoard

nav_board: NavBoard

multimedia_board: MultiMedia


def setup():
    """
    Sets up all of the interfaces
    """
    this.nav_board = NavBoard()
    this.drive_board = DriveBoard()
    this.multimedia_board = MultiMedia()
