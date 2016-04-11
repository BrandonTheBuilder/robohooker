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
BAG_NAME = '../bags/2016-04-06-16-57-50.bag'
class TestFishDetective(unittest.TestCase):
    def test_hough_transform(self):
        self.bridge = CvBridge()
        bag = rosbag.Bag(BAG_NAME)
        self.images = []
        msgs = bag.read_messages()
        def get_image():
            yield self.bridge.imgmsg_to_cv2(msgs.next()[1], "bgr8")
        imgs = get_image()
        
        import IPython; IPython.embed()
        
        
    def detect_edges(self, img, low, high):
        edges = cv2.Canny(img,low,high)
        plt.subplot(121),plt.imshow(img,cmap = 'gray')
        plt.title('Original Image'), plt.xticks([]), plt.yticks([])
        plt.subplot(122),plt.imshow(edges,cmap = 'gray')
        plt.title('Edge Image'), plt.xticks([]), plt.yticks([]) 
        plt.show()


if __name__ == '__main__':
    rostest.rosrun(PKG, 'test_fish_detective', TestFishDetective)