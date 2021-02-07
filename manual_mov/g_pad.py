from inputs import get_gamepad

while 1:
    events = get_gamepad()
    for event in events:
        if event.ev_type != "Sync" and event.ev_type != "Absolute":
            print(event.ev_type, event.code, event.state)
