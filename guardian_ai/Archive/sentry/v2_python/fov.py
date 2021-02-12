import cv2

video = cv2.VideoCapture(0)

ok, frame = video.read()

height, width, channels = frame.shape

print(height,width)
