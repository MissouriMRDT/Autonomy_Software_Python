from drivers.rovecomm import RoveComm
from drivers.driveBoard import DriveBoard
from algorithms.objecttracking import ObjectTracker

tracker = ObjectTracker()
drive = DriveBoard(RoveComm())

WIDTH           = 640.0  # pixels
FIELD_OF_VIEW   = 40.0   # degrees
TARGET_DISTANCE = 0.4    # meters
RADIUS          = .063   # meters
SCALING_FACTOR  = 10.0   # pixel-meters
POWER           = 20

while True:
    ball_in_frame, center, radius = tracker.track_ball()
    if ball_in_frame:
        angle_to_ball = FIELD_OF_VIEW * ((center[0] - (WIDTH / 2)) / WIDTH)
        distance = SCALING_FACTOR / radius
        print "Distance: %f" % distance
        if distance > TARGET_DISTANCE:
            print "Moving forward: %f" % angle_to_ball
            drive.move(POWER, angle_to_ball)
        if distance < TARGET_DISTANCE:
            print "Moving backward: %f" % angle_to_ball
            drive.move(-POWER, angle_to_ball)
    else:
        print "no ball detected"
