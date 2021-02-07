import inputs

while 1:
    events = get_gamepad()
    for event in events:
        print(event.ev_type, event.code, event.state)
