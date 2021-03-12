import socket

# Socket --------------------------------------------------------------------


s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((socket.gethostname(), 2022))
s.listen(5)


clientsocket, address = s.accept()

# This would be where you would send commands to the python file
while True:
    msg = input("Type Something Here: ")
    clientsocket.send(bytes(msg,"utf-8"))