import random

from Tracker import Tracker

class Target_Queue(object):
    def __init__(self, method = None):
        self._targets = []
        self._attack_method = method

    """
    GET TARGETS
    Getter for target list
    """
    def get_targets(self):
        return self._targets

    """
    SET TARGETS
    Setter for target list
    """
    def set_targets(self, target_list):
        self._targets = target_list

    """
    ADD TARGET
    Method to add target to Target Queue
    """
    def add_target(self, clss, frame, location):
        #create a target
        #find an id for it
        # empty list case
        if self._targets == []:
            id = 1
        else:
            highest_id = None
            for target in self._targets:
                current_id = target.get_id()
                if highest_id == None or current_id > highest_id:
                    highest_id =current_id
            id = highest_id+1
        target = Target(clss,id,frame,location)
        #add target to Queue
        self._targets.append(target)

    """
    NEXT TARGET
    Method to give the next target in the list based on the
    method of attack_method
    """
    def next_target(self):
        if self._targets == []:
            return None
        else:
            return self._targets[0]


    """
    REMOVE TARGET
    Method to remove target from queue and redesignate IDs
    (if re-detect does not show target at location)
    """
    def remove_target(self, target):
        self._targets.remove(target)

    """
    UPDATE METHOD
    Update method of attack
    """
    def update_method(self, arg):
        pass

class Target(object):
    def __init__(self, clss,id,frame,location):
        self._classification = clss
        self._id = id
        self._location = location
        self._center = (location[0] + 0.5*(location[2]), location[1] + 0.5*(location[3]))
        self._tracker = self.add_tracker(frame)

    """
    GET CLASS
    Getter for target class
    """
    def get_class(self):
        return self._classification

    """
    GET ID
    Getter for target ID
    """
    def get_id(self):
        return self._id

    """
    GET LOCATION
    Getter for target location
    """
    def get_location(self):
        return self._location

    """
    SET LOCATION
    Setter for target location
    """
    def set_location(self,location):
        self._location = location
        self.set_center((location[0] + 0.5*(location[2]), location[1] + 0.5*(location[3])))

    """
    GET CENTER
    Getter for target center
    """
    def get_center(self):
        return self._center

    """
    SET CENTER
    Setter for target center
    """
    def set_center(self,center):
        self._center = center

    """
    ADD TRACKER
    Method to add tracker to target object
    """
    def add_tracker(self,frame):
        return Tracker(frame,self._location)

    """
    UPDATE TRACKER
    Method to update tracker for target object
    (change location for target)
    """
    def update_tracker(self, frame):
        ok, self._location = self._tracker.update(frame)
        return ok
