#!/usr/bin/env python
import sys
import unittest
import rostest
import rospy
import rosbag
import os
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

PKG ="test_fish_detective"
BAG_NAME = '../training/bags/out_fish.bag'
class TestFishDetective(unittest.TestCase):
    def test_hough_transform(self):
        self.bridge = CvBridge()
        bag = rosbag.Bag(BAG_NAME)
        self.images = []
        msgs = bag.read_messages()
        def get_image():
            yield self.bridge.imgmsg_to_cv2(msgs.next()[1], "bgr8")
        self.imgs = get_image()
        #self.detect_edges(20,40)
        # self.mouse_events()
        self.find_colours()
        
        
        
        
    def detect_edges(self, low, high):
        img = self.imgs.next()
        edges = cv2.Canny(img,low,high)
        plt.subplot(121),plt.imshow(img,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([]) 
        plt.show()


    def mouse_events(self):
        img = self.imgs.next()
        self.circles = []
        cv2.namedWindow("Calibration")
        cv2.startWindowThread()
        cv2.setMouseCallback("Calibration", self.mouse_press)
        cv2.imshow('Calibration', img)
        k = cv2.waitKey(0)
        for circle in self.circles:
            cv2.circle(img,circle,10,(255,255,255),3)
        cv2.imshow('Calibration', img)
        k = cv2.waitKey(0)
        cv2.destroyWindow('Calibration')


    def find_colours(self):
        f_name = 'TestImg.png'

        img = self.imgs.next()
        cv2.imwrite(f_name, img) 
        self.pick_color_img = img
        cv2.namedWindow("Pick a colour")
        cv2.startWindowThread()
        cv2.setMouseCallback("Pick a colour", self.color_press)
        cv2.imshow("Pick a colour", img)
        k = cv2.waitKey(0)
        cv2.destroyWindow("Pick a colour")

    def color_press(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            import IPython; IPython.embed()

    def mouse_press(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print 'Click at {},{}!'.format(x,y)
            self.circles.append((x,y))
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.fish_center[self.calibration_index] = Point(x,-y)



if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_fish_detective', TestFishDetective)