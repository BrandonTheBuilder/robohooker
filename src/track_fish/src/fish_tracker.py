from point import Point
import numpy as np

class FishTracker(object):
    def __init__(self):
        pass

    def calibrate(self, board_center, fish_centers, t):
        """
        """
        self.board_center = board_center
        radii = []
        theta = []
        for point in fish_centers:
            pos = point-self.board_center
            radii.append(pos.magnitude())
            theta.append(pos.angle())

        self.radius = np.mean(radii)
        rates = []
        for i in range(1, len(t)-1):
            dTheta = theta[i]-theta[i-1]
            dt = t[i]-t[i-1]
            rates.append(dTheta/dt)
        rate = np.mean(rates)
        freq = rate/(2*np.pi)
        phase = np.arcsin((theta[0]-np.pi)/np.pi) - freq*t[0]
        self.find_angle = lambda _t: (np.pi + np.pi*np.sin(freq*_t+phase)) 
        self.calibrated = True