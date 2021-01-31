import serial
arduino = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
arduino.println(1)
