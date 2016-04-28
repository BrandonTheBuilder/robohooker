from point import Point
from collections import deque
import numpy as np
import cv2
DEQUE_SIZE = 30
CASCADE_CLASSIFIER = '../../fish_detective/training/harr_fish/cascade.xml'

HARR_CUTOFF = 1
# ANGLE_TOLERANCE = 0.01
LOCATION_TOLERANCE = 10

class FishHole(object):
    """docstring for FishHole"""
    def __init__(self):
        super(FishHole, self).__init__()
        self.t = deque()
        self.fish_center = deque()
        self.rate = 0
        self.offset = 0
        self.calibrated = False
        self.find_fish = cv2.CascadeClassifier(CASCADE_CLASSIFIER)
        self.fishiness = deque()


    def get_position(self, t):
        theta = self.rate*t + self.offset
        return Point(self.radius, theta, from_polar=True)


    def append(self, l, x):
        if len(l) < DEQUE_SIZE:
            l.append(x)
        else:
            l.popleft()
            self.append(l,x)

    def update(self, point, img, t):
        location = self.get_position(t)
        if abs(location.x-point.x)>LOCATION_TOLERANCE and abs(location.y-point.y)>LOCATION_TOLERANCE:
            return False
        else:
            self.calibrate(point, t) 
            self.fishy_calibration(img)
            return True


    def fishy_calibration(self, img):
        fishes = self.find_fish.detectMultiScale(img, 1.01, 1, 
                                                minSize = (25,25), 
                                                maxSize=(35,35))
        self.append(self.fishiness, len(fishes))


    def is_fish(self):
        fishy_level = np.mean(self.fishiness)
        if fishy_level < HARR_CUTOFF:
            return False
        else:
            return True


    def calibrate(self, fish_center, t):
        """
        """
        self.append(self.fish_center, fish_center)
        self.append(self.t, t)
        if len(self.t) < 2:
            print 'Calibrating'
            self.radius = fish_center.magnitude()
            self.rate = 0
            self.offset = fish_center.angle()
        else:
            radii = []
            theta = []
            for point in self.fish_center:
                pos = point
                radii.append(pos.magnitude())
                theta.append(pos.angle())

            self.radius = np.mean(radii)
            rates = []
            for i in range(len(self.t)):
                if i >= 1:
                    dTheta = theta[i]-theta[i-1]
                    dt = self.t[i]-self.t[i-1]
                    rates.append(dTheta/dt)
            rate = np.mean(rates)
            bs = []
            for i in range(0, len(self.t)-1):
                bs.append(theta[i] - rate*self.t[i])
            b = np.mean(bs)
            self.rate = rate
            self.offset = b
            self.calibrated = True
        