import numpy as np
from numpy.linalg import inv

__all__ = ['Arm']

STEP_SIZE = 0.01
TOLERANCE = 0.05
SCALAR = 1.0

def Rx(theta):
    return np.matrix([[1, 0, 0],
                     [0, np.cos(theta), -np.sin(theta)],
                     [0, np.sin(theta), np.cos(theta)]])

def Ry(theta):
    return np.matrix([[np.cos(theta), 0, np.sin(theta)],
                     [0, 1, 0],
                     [-np.sin(theta), 0, np.cos(theta)]])

def Rz(theta):
    return np.matrix([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])

class Arm(object):
    rao = np.matrix([0, 0, 4.0]).T
    rba = np.matrix([0, 0, 3.5]).T
    theta = [0, 0, 0]

    def __init__(self):
        pass


    def end_position(self):
        return Rz(self.theta[0])*Ry(self.theta[1])*(self.rao + Ry(self.theta[2])*self.rba)
        

    def a_position(self):
        return Rz(self.theta[0])*Ry(self.theta[1])*self.rao

    def jacobian(self):
        rz = np.matrix([0, 0, 1])
        ry = np.matrix([0,1,0])

        a = np.cross(rz, self.end_position().T)
        r_b = Rz(self.theta[0])*ry.T
        b = np.cross(r_b.T, self.end_position().T)
        r_c = Rz(self.theta[0])*ry.T
        c = np.cross(r_c.T, self.a_position().T)
        return np.concatenate((a,b,c)).T

    def get_theta(self):
        def reduce_angle(theta):
            if theta >= np.pi*2:
                return reduce_angle(theta - np.pi*2)
            elif theta <= -np.pi*2:
                return reduce_angle(theta + np.pi*2)
            elif theta < 0:
                return (np.pi+theta) + np.pi
            else:
                return theta
        return [reduce_angle(self.theta[0]), reduce_angle(self.theta[1]), reduce_angle(self.theta[2])]

    def plan_path(self, goal):
        done = False
        rz = np.matrix([0, 0, 1])
        ry = np.matrix([0,1,0])

        while not done:
            e = self.end_position()
            dx = 0
            dy = 0
            dz = 0
            if abs(e.A1[0]-goal[0]) > TOLERANCE:
                dx = (goal[0]-e.A1[0])
                
            if abs(e.A1[1]-goal[1]) > TOLERANCE:
                dy = (goal[1]-e.A1[1])
                
            if abs(e.A1[2]-goal[2]) > TOLERANCE:
                dz = (goal[2]-e.A1[2])
                
            if dx == 0 and dy == 0 and dz == 0: 
                done = True

            de = [dx, dy, dz]
            a = np.cross(rz, e.T)
            r_b = Rz(self.theta[0])*ry.T
            b = np.cross(r_b.T, self.end_position().T)
            r_c = Rz(self.theta[0])*ry.T
            c = np.cross(r_c.T, self.a_position().T)
            jac = np.concatenate((a,b,c)).T
            try:
                inv_jac = inv(jac)
            except:
                inv_jac = jac.T
            dTheta = inv_jac*np.matrix(de).T
            if not done and np.array_equiv(dTheta.A1, [0,0,0]):
                dTheta = np.matrix([0.00, 0.01, -0.01]).T
                
            theta = np.matrix(self.theta).T + SCALAR*dTheta
            self.theta = theta.T.A1
        return self.theta

