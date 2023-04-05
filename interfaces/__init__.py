#
# Mars Rover Design Team
# __init_.py
#
# Created on Jul 19, 2020
# Updated on Aug 21, 2022
#

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


def setup(relative_positioning="DISABLE"):
    """
    Sets up all the interfaces
    """
    this.nav_board = NavBoard(relative_positioning)
    this.drive_board = DriveBoard()
    this.multimedia_board = MultiMedia()
