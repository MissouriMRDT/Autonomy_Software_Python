from enum import Enum


# Performs a search pattern based on the provided argument
def search_for_marker(search_pattern):
    if search_pattern == SearchPattern.ARCHIMEDES:
        archimedes_spiral_search()
    elif search_pattern == SearchPattern.SQUARE:
        square_spiral_search()
    else:
        print("Illegal argument in search_for_marker: Must be enum of SearchPattern")


# Drives in an archimedes spiral
def archimedes_spiral_search():
    pass


# Drives in a square spiral
def square_spiral_search():
    pass


class SearchPattern(Enum):
    ARCHIMEDES = 1
    SQUARE = 2
