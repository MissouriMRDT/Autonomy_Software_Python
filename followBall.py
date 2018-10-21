from drivers.rovecomm import RoveComm
from drivers.drive_board import DriveBoard
from algorithms.objectTracking import ObjectTracker
import constants

tracker = ObjectTracker()
drive = DriveBoard(RoveComm())

while True:
    ball_in_frame, center, radius = tracker.track_ball()

    if ball_in_frame:
        angle_to_ball = FIELD_OF_VIEW * ((center[0] - (WIDTH / 2)) / WIDTH)
        distance = SCALING_FACTOR / radius
        print("Distance: %f" % distance)

        if distance > TARGET_DISTANCE:
            print("Moving forward: %f" % angle_to_ball)
            left, right = drive.calculate_move(POWER, angle_to_ball)
            drive.send_drive(left, right)

        if distance < TARGET_DISTANCE:
            print("Moving backward: %f" % angle_to_ball)
            left, right = drive.calculate_move(-POWER, angle_to_ball)
            drive.send_drive(left, right)

    else:
        print("no ball detected")
