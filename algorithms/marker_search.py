from drivers.drive_board import DriveBoard


# Performs a search pattern based on the provided argument
def search_for_marker(drive):
    drive.calculate_move(1, 1)
