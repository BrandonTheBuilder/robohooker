import unittest
import numpy as np
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from point import Point
from fish_tracker import FishTracker

class TestFishTracker(unittest.TestCase):
    def test_point(self):
        a = Point(5,5)
        b = Point(1,3)
        c = Point(6,8)
        d = Point(4,2)
        self.assertEquals(a+b, c)
        self.assertEquals(a-b, d)
        quadrant_one = Point(3,3)
        self.assertAlmostEquals(np.pi/4, quadrant_one.angle())
        quadrant_two = Point(-3,3)
        self.assertAlmostEquals((3*np.pi)/4, quadrant_two.angle())
        quadrant_three = Point(-3,-3)
        self.assertAlmostEquals((5*np.pi)/4, quadrant_three.angle())
        quadrant_four = Point(3,-3)
        self.assertAlmostEquals((7*np.pi)/4, quadrant_four.angle())
        pos_x = Point(3, 0)
        self.assertAlmostEquals(0, pos_x.angle())
        pos_y = Point(0, 3)
        self.assertAlmostEquals(np.pi/2, pos_y.angle())
        neg_x = Point(-3, 0)
        self.assertAlmostEquals(np.pi, neg_x.angle())
        neg_y = Point(0, -3)
        self.assertAlmostEquals(3*np.pi/2, neg_y.angle())
        a = Point(3,3)
        b = Point(5,3)
        c = b-a
        self.assertAlmostEquals(0, c.angle())
        quadrant_one_polar = Point(quadrant_one.magnitude(),
                                   quadrant_one.angle(), 
                                   from_polar=True)
        self.assertAlmostEquals(quadrant_one_polar.x, quadrant_one.x)
        self.assertAlmostEquals(quadrant_one_polar.y, quadrant_one.y)

        quadrant_two_polar = Point(quadrant_two.magnitude(),
                                   quadrant_two.angle(), 
                                   from_polar=True)
        self.assertAlmostEquals(quadrant_two_polar.x, quadrant_two.x)
        self.assertAlmostEquals(quadrant_two_polar.y, quadrant_two.y)

        quadrant_three_polar = Point(quadrant_three.magnitude(),
                                     quadrant_three.angle(), 
                                     from_polar=True)
        self.assertAlmostEquals(quadrant_three_polar.x, quadrant_three.x)
        self.assertAlmostEquals(quadrant_three_polar.y, quadrant_three.y)

        quadrant_four_polar = Point(quadrant_four.magnitude(),
                                    quadrant_four.angle(), 
                                    from_polar=True)
        self.assertAlmostEquals(quadrant_four_polar.x, quadrant_four.x)
        self.assertAlmostEquals(quadrant_four_polar.y, quadrant_four.y)


    def test_fish_tracker(self):
        r = 20
        center = Point(356, 412)
        points = []
        t =[]
        for i in range(8):
            points.append(Point(r, (np.pi-np.pi*(i)/8)+np.pi, from_polar=True))
            t.append(i*0.2)
        theta_one = (np.pi-np.pi/8)+np.pi
        self.assertEquals(points[1].angle(), theta_one)
        for i in range(8):
            points[i]=points[i]+center
        for point in points:
            print point
        ft = FishTracker()
        ft.calibrate(center, points[0:3], t[0:3])
        for i in range(3,8):
            fish = Point(ft.radius, ft.find_angle(t[i]))+center
            self.assertEquals(abs(fish.x-points[i].x)>1, True)
            self.assertEquals(abs(fish.y-points[i].y)>1, True)


