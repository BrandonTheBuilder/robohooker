#!/usr/bin/env python
import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class Hough(object):

  def __init__(self):
    self.bridge = CvBridge()
    self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=2)
    self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)

  def callback(self, image):
    cimg = self.bridge.imgmsg_to_cv2(image, "bgr8")
    img = cv2.cvtColor(cimg, cv2.COLOR_BGR2GRAY)
    cimg = img
    # img = cv2.medianBlur(img,5)

    circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,70,param1=2,
              param2=5, minRadius=25, maxRadius=40)

    circles = np.uint16(np.around(circles))
    for i in circles[0,:]:
        # draw the outer circle
        cv2.circle(cimg,(i[0],i[1]),i[2],(0,255,0),2)
        # draw the center of the circle
        cv2.circle(cimg,(i[0],i[1]),2,(0,0,255),3)

    try:
      self.image_pub.publish(self.bridge.cv2_to_imgmsg(cimg, "8UC1"))
    except CvBridgeError as e:
      print(e)


    

def main(args):
  rospy.init_node('fish_detective')
  h = Hough()
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)