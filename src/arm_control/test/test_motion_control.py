import unittest
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from src.motion_control import Arm
import numpy as np
import time

class TestMotionControl(unittest.TestCase):
    def test_arm_position(self):
        a = Arm()
        e = np.matrix([[0,0,7.5]]).T
        self.assertEquals(np.array_equiv(a.end_position(), e), True)
        # rotation about theta and phi
        a.theta = [np.pi/2, np.pi/2, 0]
        e = np.matrix([[0,7.5,0]]).T
        self.assertEquals(np.allclose(a.end_position(), e, atol=1E-4), True)
        # Rotation about psi
        a.theta = [0, 0, np.pi/2]
        e = np.matrix([[3.5,0,4.0]]).T
        self.assertEquals(np.allclose(a.end_position(), e, atol=1E-4), True)

    def test_jacobian(self):
        a = Arm()
        theta = np.matrix([1.2, 0.23, 0.69]).T
        dtheta = np.matrix([0.01, 0.01, 0.01]).T
        a.theta = theta.T.A1
        dE = a.jacobian()*dtheta
        c = a.jacobian()
        e_one = a.end_position()
        theta_two = theta + dtheta
        a.theta = theta_two.T.A1
        e_est = e_one + dE
        try:
            self.assertEquals(np.allclose(a.end_position(), e_est, atol=1E-1), True)
        except Exception, e:
            print '\nEstimated: {} \nActual: {}\n'.format(e_est, a.end_position())

    def test_plan_path(self):
        a = Arm()
        goal = [0.0, 0.0, 7.0]
        start = time.time()
        theta = a.plan_path(goal)
        end = time.time()
        print 'found {} in {} secs'.format(a.get_theta(), end-start)
        self.assertEquals(np.allclose(a.end_position(), np.matrix(goal).T, atol=1E-1), True)
        goal = [0.0, 0.0, 6.0]
        start = time.time()
        theta = a.plan_path(goal)
        end = time.time()
        print 'found {} in {} secs'.format(a.get_theta(), end-start)
        self.assertEquals(np.allclose(a.end_position(), np.matrix(goal).T, atol=1E-1), True)
        goal = [0.0, 0.0, 7.5]
        start = time.time()
        theta = a.plan_path(goal)
        end = time.time()
        print 'found {} in {} secs'.format(a.get_theta(), end-start)
        self.assertEquals(np.allclose(a.end_position(), np.matrix(goal).T, atol=1E-1), True)
        goal = [2.0, 1.0, 5.5]
        start = time.time()
        theta = a.plan_path(goal)
        end = time.time()
        print 'found {} in {} secs'.format(a.get_theta(), end-start)
        self.assertEquals(np.allclose(a.end_position(), np.matrix(goal).T, atol=1E-1), True)
        
        