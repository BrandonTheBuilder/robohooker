from point import Point
from collections import deque
import numpy as np
import cv2
DEQUE_SIZE = 30
CASCADE_CLASSIFIER = '../../fish_detective/training/harr_fish/cascade.xml'

HARR_CUTOFF = 1
# ANGLE_TOLERANCE = 0.01
LOCATION_TOLERANCE = 100

class FishHole(object):
    """docstring for FishHole"""
    def __init__(self):
        super(FishHole, self).__init__()
        self.find_fish = cv2.CascadeClassifier(CASCADE_CLASSIFIER)
        self.fishiness = deque()


    def append(self, l, x):
        if len(l) < DEQUE_SIZE:
            l.append(x)
        else:
            l.popleft()
            self.append(l,x)


    def fishy_calibration(self, img):
        # fishes = self.find_fish.detectMultiScale(img, 1.01, 1, 
        #                                         minSize = (25,25), 
        #                                         maxSize=(35,35))
        # self.append(self.fishiness, len(fishes))
        self.append(self.fishiness, 2)
        pass


    def is_fish(self):
        fishy_level = np.mean(self.fishiness)
        if fishy_level < HARR_CUTOFF:
            return False
        else:
            return True


    
        