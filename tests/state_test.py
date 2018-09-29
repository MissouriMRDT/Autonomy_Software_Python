import rover_states as rs
import time


def callback_func():
    print("Running callback")
    time.sleep(TIME_INTERVAL)
    print("Switched to state: " + str(stateSwitcher.state) + "\n")


stateSwitcher = rs.StateSwitcher()
TIME_INTERVAL = 2

time.sleep(TIME_INTERVAL)

print("Starting autonomy...")
stateSwitcher.handle_event(rs.AutonomyEvents.START)
print("Switched to state: " + str(stateSwitcher.state) + "\n")

time.sleep(TIME_INTERVAL)

print("Rover reached a marker")
stateSwitcher.handle_event(rs.AutonomyEvents.REACHED_MARKER, callback=callback_func)

time.sleep(TIME_INTERVAL)

print("Test terminated")

counter = 0

while True:

    if str(stateSwitcher.state) == str(rs.Idle()):
        print("waiting")
        counter += 1

        if counter > 10:
            stateSwitcher.handle_event(rs.AutonomyEvents.START)
    elif str(stateSwitcher.state) == str(rs.Navigating()):
        print("Rovin")
        counter -= 2

        if counter < 0:
            stateSwitcher.handle_event(rs.AutonomyEvents.ABORT)

    elif str(stateSwitcher.state) == str(rs.Shutdown()):
        stateSwitcher.handle_event(rs.AutonomyEvents.RESTART)

    time.sleep(1)




