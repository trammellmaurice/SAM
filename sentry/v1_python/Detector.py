import jetson.inference
import jetson.utils
import cv2

class Detector(object):
	def __init__(self, thr, model):
		self.net = jetson.inference.detectNet(model, threshold=thr)

	"""
	SCAN
	Method to scan for targets 
	(CONVERT IMAGE TO CUDA)
	"""
	def scan(self, img):
		img = jetson.utils.cudaFromNumpy(img) 
		return self.net.Detect(img)		

class Detection(object):
    def __init__(self,location,type = "person"):
        self.type = type
        self.location = location
        self.center = (location[0] + 0.5*(location[2]), location[1] + 0.5*(location[3]))

if __name__ == '__main__':
	#camera = jetson.utils.videoSource("/dev/video0")      # '/dev/video0' for V4L2
	camera = cv2.VideoCapture(0)
	display = jetson.utils.videoOutput("display://0") # 'my_video.mp4' for file
	d = Detector(0.5,"ssd-mobilenet-v2")
	while display.IsStreaming():
		ok, img = camera.read()
		detections = d.scan(img)
		if detections != []:
			print(detections[0].Left,detections[0].Top,detections[0].Width,detections[0].Height)
		img = jetson.utils.cudaFromNumpy(img)
		display.Render(img)
