from rover_states import StateSwitcher, AutonomyEvents
import time


def callback_func():
    print "Running callback"
    time.sleep(TIME_INTERVAL)
    print "Switched to state: " + str(stateSwitcher.state) + "\n"


stateSwitcher = StateSwitcher()
TIME_INTERVAL = 5

time.sleep(TIME_INTERVAL)

print "Starting autonomy..."
stateSwitcher.handle_event(AutonomyEvents.START)
print "Switched to state: " + str(stateSwitcher.state) + "\n"

time.sleep(TIME_INTERVAL)

print "Rover reached a marker"
stateSwitcher.handle_event(AutonomyEvents.REACHED_MARKER, callback=callback_func)

time.sleep(TIME_INTERVAL)

print("Test terminated")



