import rover_states as rs

state_switcher = rs.StateSwitcher()

while True:

    if state_switcher.state == rs.Idle():
        pass

    elif state_switcher.state == rs.Navigating():
        pass

    elif state_switcher.state == rs.Searching():
        pass

    elif state_switcher.state == rs.ApproachingMarker():
        pass

    elif state_switcher.state == rs.Shutdown():
        pass
