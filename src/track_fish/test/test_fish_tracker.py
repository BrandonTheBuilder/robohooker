import unittest
from unittest import skip
import numpy as np
import sys, os
import cv2
import cv2.cv as cv
from cv_bridge import CvBridge
import rosbag
import random
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from point import Point
import fish_hole as fsh
from fish_hole import FishHole
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


    def test_append(self):
        ft = FishHole()
        for i in range(fsh.DEQUE_SIZE + 3):
            ft.append(ft.fishiness, i)
        for i in range(fsh.DEQUE_SIZE):
            self.assertEquals(ft.fishiness[i], i+3)


    def test_fishy_calibration(self):
        fh = FishHole()
        dirname = os.path.dirname(__file__)
        neg = os.path.join(dirname,
            '../../fish_detective/training/negative/')
        pos = os.path.join(dirname,
            '../../fish_detective/training/close/')
        poss = os.walk(pos)
        negs = os.walk(neg)
        p_root, dirs, pos_files = poss.next()
        n_root, dirs, neg_files = negs.next()

        BAG_NAME = '../../fish_detective/bags/2016-04-06-16-57-50.bag' #1437
        bag = rosbag.Bag(BAG_NAME)
        imgs = self.get_img(bag)
        imgs = [imgs.next() for i in range(100)]
        for img in imgs:
            fh.fishy_calibration(img)
        self.assertEquals(fh.is_fish(), True)
        BAG_NAME = '../../fish_detective/training/bags/no_fish.bag'
        bag = rosbag.Bag(BAG_NAME)
        imgs = self.get_img(bag)
        imgs = [imgs.next() for i in range(100)]
        neg_count = 1
        for img in imgs:
            fh.fishy_calibration(img)
            # print 'neg_count {}, Is fish: {}'.format(neg_count,
            #                                         fh.is_fish())
            neg_count+=1
        self.assertEquals(False, fh.is_fish())



    def get_img(self, bag):
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages(topics=['/cv_camera/image_raw/']):
            img = bridge.imgmsg_to_cv2(msg, "bgr8")
            rects = self.getCircles(img, 35)
            for rect in rects:
                yield rect

    def get_raw_img(self, bag):
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages(topics=['/cv_camera/image_raw/']):
            img = bridge.imgmsg_to_cv2(msg, "bgr8")
            yield img

    def get_ros_img(self, bag):
        bridge = CvBridge()
        for topic, msg, t in bag.read_messages(topics=['/cv_camera/image_raw/']):
            yield msg


    def getCircles(self, img, rad):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,70,param1=10,
              param2=40, minRadius=25, maxRadius=35)
        circles = np.uint16(np.around(circles))
        rects = []
        for i in circles[0,:]:
            rects.append(img[i[1]-rad:i[1]+rad, i[0]-rad:i[0]+rad])
        return rects

    def test_rotation(self):
        ft = FishTracker()
        r = 20
        points = [Point(r, 0.0, from_polar=True)]
        p = [n.get_tuple() for n in points]
        rot = ft.rotate(p, np.pi/4)
        rotated = Point(*rot[0])
        self.assertEquals(rotated.angle(), np.pi/4)

        points = [Point(r, 0.0, from_polar=True), 
                    Point(r, np.pi/4, from_polar=True)]
        p = [n.get_tuple() for n in points]
        rot = ft.rotate(p, np.pi/4)
        rot_one = Point(*rot[0])
        rot_two = Point(*rot[1])
        self.assertAlmostEquals(rot_one.angle(), np.pi/4)
        self.assertAlmostEquals(rot_two.angle(), np.pi/2)

    
    def test_list_compare(self):
        ft = FishTracker()
        r = 20
        est = (np.pi/4)
        points = []
        offset = []
        for i in range(5):
            points.append(Point(r, (np.pi-np.pi*(i)/8)+np.pi, from_polar=True))
        
        ref = [a.get_tuple() for a in points]
        full_comp = ft.rotate(ref, -np.pi/4)
        comp = []
        for i in range(3):
            choice = random.choice(full_comp)
            full_comp.remove(choice)
            comp.append(choice)

        
        full_comp = ft.rotate(ref, -np.pi/4)
        e, r, matching = ft.compare_lists(full_comp, comp)
        self.assertEquals(matching, True)
        for key in e.keys():
            self.assertEquals(full_comp[key], e[key][1])


    def test_get_angle_offset(self):
        ft = FishTracker()
        angle = -np.pi/4
        r = 20
        est = (angle + 0.25)
        points = []
        offset = []
        for i in range(5):
            points.append(Point(r, (np.pi-np.pi*(i)/8)+np.pi, from_polar=True))
        
        ref = [a.get_tuple() for a in points]
        full_comp = ft.rotate(ref, angle)
        comp = []
        for i in range(3):
            choice = random.choice(full_comp)
            full_comp.remove(choice)
            comp.append(choice)

        
        full_comp = ft.rotate(ref, angle)
        e, r = ft.get_angle_offset(ref, comp, est, denom=8)
        self.assertAlmostEqual(angle, e, places = 2)
        


    @skip("This test is a visual confirmation")
    def test_calibration(self):
        ft = FishTracker()
        BAG_NAME = '../../fish_detective/bags/2016-04-06-16-57-50.bag' #1437
        bag = rosbag.Bag(BAG_NAME)
        imgs = self.get_ros_img(bag)
        img = imgs.next()
        cropped = ft.crop_img(img)
        center = ft.board_center
        cv2.circle(cropped, center.to_image(), 3, (255,0,0), 3)
        fishes = [Point(*fish) for fish in ft.fish_locales]
        for fish in fishes:
            glob_fish = center+fish
            cv2.circle(cropped, glob_fish.to_image(),30,(255,0,0),3)
        cv2.namedWindow("Calibration")
        cv2.startWindowThread()
        cv2.imshow('Calibration', cropped)
        k = cv2.waitKey(0)
        cv2.destroyWindow('Calibration')


    def test_find_holes(self):
        ft = FishTracker()
        BAG_NAME = '../../fish_detective/bags/2016-04-06-16-57-50.bag' #1437
        bag = rosbag.Bag(BAG_NAME)
        imgs = self.get_ros_img(bag)
        zero = imgs.next()
        # import IPython; IPython.embed()
        fishes = ft.find_holes(zero)
        cv2.namedWindow("Calibration")
        cv2.startWindowThread()
        cv2.imshow('Calibration', fishes)
        k = cv2.waitKey(0)
        one = imgs.next()
        fishes = ft.find_holes(one)
        cv2.imshow('Calibration', fishes)
        k = cv2.waitKey(0)
        cv2.destroyWindow('Calibration')




