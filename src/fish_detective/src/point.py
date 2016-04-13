import numpy as np

class Point(object):
    def __init__(self, x, y, from_polar=False):
        # from polar r=x, theta=y
        if from_polar:
            self.x = np.cos(y)*x
            self.y = np.sin(y)*x
        else:
            self.x = x
            self.y = y

    def angle(self):
        angle =  np.arctan2(self.y,self.x)
        if angle > 0:
            return angle
        else:
            angle = (np.pi+angle) + np.pi
        if angle >= np.pi*2:
            return angle - np.pi*2
        else:
            return angle

    def magnitude(self):
        return np.sqrt(self.x**2 + self.y**2)

    def __add__(self, other):
        return Point(self.x + other.x, self.y+other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __cmp__(self, other):
        if self.x == other.x and self.y == other.y:
            return 0

    def __str__(self):
        return '{},{}'.format(self.x, self.y)
