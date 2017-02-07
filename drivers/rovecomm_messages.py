from rovecomm import RoveCommMessage

class AddWaypoint(RoveCommMessage):
    data_id = 666
    format = '>dd'

class ClearWaypoints(RoveCommMessage):
    data_id = 667
    format = None

class WaypointReached(RoveCommMessage):
    data_id = 668
    format = '>dd'

class EnableAutonomy(RoveCommMessage):
    data_id = 669
    format = None

class DisableAutonomy(RoveCommMessage):
    data_id = 670
    format = None

class AutonomyStatus(RoveCommMessage):
    data_id = 671
    format = '>b'

    # Define a custom send here that encodes state
