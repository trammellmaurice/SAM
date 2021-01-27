import cv2

class Tracker(object):
    def __init__(self, frame, location):
        self._model = cv2.legacy.TrackerMOSSE_create()
        ok = self._model.init(frame, location)

    """
    UPDATE TRACKER
    Method to update the tracker
    Return the new target location and ok
    """
    def update(self, frame):
        ok, new_location = self._model.update(frame)
        return ok, new_location
