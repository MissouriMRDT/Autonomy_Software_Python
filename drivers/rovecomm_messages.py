from rovecomm import RoveCommMessage

class AddWaypoint(RoveCommMessage):
    data_id = 2578
    format = '>dd'

class ClearWaypoints(RoveCommMessage):
    data_id = 2579
    format = None

class WaypointReached(RoveCommMessage):
    data_id = 2580
    format = '>dd'

class EnableAutonomy(RoveCommMessage):
    data_id = 2576
    format = None

class DisableAutonomy(RoveCommMessage):
    data_id = 2577
    format = None
    
class AutonomyStatus(RoveCommMessage): #must be implemented in red
    data_id = 2581
    format = '>b'

    # Define a custom send here that encodes state
