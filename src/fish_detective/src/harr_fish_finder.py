#!/usr/bin/env python
import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

CASCADE_CLASSIFIER = '../training/harr_fish/cascade.xml'

class HaarFishFinder(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=2)
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
        self.find_fish = cv2.CascadeClassifier(CASCADE_CLASSIFIER)

    def callback(self, image):
        cimg = self.bridge.imgmsg_to_cv2(image, "bgr8")
        img = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
        fishes = self.find_fish.detectMultiScale(img, 1.01, 0)
        rospy.loginfo('Found {} Fishes'.format(len(fishes)))
        for (x,y,w,h) in fishes:
            cv2.rectangle(img,(x,y),(x+w,y+h),(255,0,0),2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))


def main(args):
  rospy.init_node('fish_detective')
  hff = HaarFishFinder()
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)