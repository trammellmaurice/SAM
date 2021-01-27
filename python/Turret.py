import cv2
import math
from Target_Queue import *
from Detector import *
from Banner import Banner



class Turret(object):
    def __init__(self):
        self.camera = cv2.VideoCapture(0)
        self.detector = Detector(0.5,"ssd-mobilenet-v2")
        self.target_queue = Target_Queue("Random")
        self.banner = Banner()
        self.banner.show()

    """
    LOOK
    Method to get a frame from the camera
    """
    def look(self):
        ok, frame = self.camera.read()
        if ok:
            return frame
        else:
            return None

    """
    DETECT TARGETS
    Method to detect targets and return list of detections
    """
    def detect_targets(self,frame):
        #call detector scan
        detections = self.detector.scan(frame)
        #compare detections to targets
        detections = self.compare_targets(detections)
        #add leftover detections
        for detection in detections:
            self.target_queue.add_target(detection.ClassID,frame,(round(detection.Left),round(detection.Top),round(detection.Width),round(detection.Height)))

    """
    TRACK TARGETS
    Method to update trackers
    """
    def track_targets(self, frame):
        result = []
        for target in self.target_queue.get_targets():
            result.append(target.update_tracker(frame))
        return result

    """
    COMPARE TARGETS
    Method to compare new detections with IDd targets in Queue
    Cases:
    A -  No targets in queue, all new detections
    B - All targets in queue are redetected
    C - Some targets in queue are redetected
    D - No targets in queue are redetected
    """
    def compare_targets(self, detections):
        verified_targets = []
        for detection in detections:
            # find the closest target
            threshold = 40
            closest_target = None
            targets = self.target_queue.get_targets()
            if targets != []:
                for target in targets:
                    distance = _find_distance(detection.Center,target.get_center())
                    if distance < threshold:
                        threshold = distance
                        closest_target = target
                if closest_target != None:
                    closest_target.set_location((detection.Left,detection.Top,detection.Width,detection.Height))
                    detections.remove(detection) # remove detection from unIDd list
                    verified_targets.append(closest_target)
                #erase old targets and add only updated targets
        self.target_queue.set_targets(verified_targets)
        # return the unIDd detection list
        return detections

    """
    MARK TARGETS
    Method to mark targets
    """
    def mark_targets(self,current_target,img):
        height, width, channels = img.shape
        for target in self.target_queue.get_targets():
            p1 = (int(target._location[0]), int(target._location[1]))
            p2 = (int(target._location[0] + target._location[2]), int(target._location[1] + target._location[3]))
            if target.get_id() == current_target.get_id():
                cv2.rectangle(img, p1, p2, (0,0,255), 2, 1)
            else:
                cv2.rectangle(img, p1, p2, (255,0,0), 2, 1)
        # draw crosshair
        cv2.line(img, (round(width/2-30),round(height/2)), (round(width/2+30),round(height/2)), (0,0,255), 2)
        cv2.line(img, (round(width/2),round(height/2-30)), (round(width/2),round(height/2+30)), (0,0,255), 2)


    """
    NEXT TARGET
    Method to pick (based on attack method) a target to engage
    """
    def next_target(self):
        return self.target_queue.next_target()

    """
    ENGAGE TARGET
    Method that: decides movement to next target or to fire
    """
    def engage_target(self, current_target,height,width):
        x = round(width/2)
        y = round(height/2)
        target = current_target.get_location()
        if x < target[0]+target[2] and x > target[0] and y < target[1]+target[3] and y > target[1]:
            return True
"""
FIND DISTANCE
Function to find distance between two center points
"""
def _find_distance(detection,target):
    return math.sqrt( ((detection[0]-target[0])**2)+((detection[1]-target[1])**2))

if __name__ == '__main__':
    #Testing script
    turret = Turret()
    while True:
        img = turret.look() # take a frame
        height,width,channels = img.shape
        #cv2.imshow("Turret",img) #display frame
        # while True:
        #     # Exit if ESC pressed
        #     k = cv2.waitKey(1) & 0xff
        #     if k == 27 : break
        turret.detect_targets(img)
        CYCLE = 200 * len(turret.target_queue.get_targets())
        # print(turret.target_queue._targets[0]._id)
        i = clock = 0
        current_target = None
        for i in range(0,CYCLE): #tracking loop
            img = turret.look()
            ok = turret.track_targets(img)
            current = turret.next_target()
            if current != None and turret.engage_target(current,height,width):
                print("SHOOT")
                clock +=1
                if clock == 25:
                    turret.target_queue.remove_target(current)
                    clock = 0
                    print("NEXT")
            turret.mark_targets(current,img)
            # Display result
            cv2.imshow("TURRETCAM", img)

            # Exit if ESC pressed
            k = cv2.waitKey(1) & 0xff
            if k == 27 : break
