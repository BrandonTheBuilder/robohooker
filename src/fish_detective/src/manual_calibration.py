#!/usr/bin/env python
import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
BUFFER_SIZE = 10
TIME_STEP = 0.5

class HandFishFinder(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=2)
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
        self.calibration_buffer = []
        self.buffer_full = False
        self.calibrated = False


    def add_image(self, image):
        t = rospy.get_time()
        buffer_len = len(self.calibration_buffer)
        if buffer_len == 0:
            self.calibration_buffer.append((t,image))
        elif buffer_len < BUFFER_SIZE:
            last = self.calibration_buffer[buffer_len-1][0]
            if t-last >= TIME_STEP:
                self.calibration_buffer.append((t,image))
        if len(self.calibration_buffer) == BUFFER_SIZE:
            self.buffer_full = True 

    def calibrate(self):
        cv2.namedWindow("Calibration")
        cv2.startWindowThread()
        cv2.setMouseCallback("Calibration", self.mouse_press)
        while not self.buffer_full:
            time.sleep(0.5)
        self.calibration_index = 0
        self.board_center = [(0,0)]*BUFFER_SIZE
        self.fish_center = [(0,0)]*BUFFER_SIZE
        t_steps = []
        for img in self.calibration_buffer:
            t_steps.append(img[0])
            self.calibration_phase = 'board'
            cv2.imshow('Calibration', img[1])
            k = cv2.waitKey(0)
            self.calibration_index += 1
        cv2.destroyWindow('Calibration')
        # Now that we have some data let's use it
        x = np.mean([point[0] for point in self.board_center])
        y = np.mean([point[1] for point in self.board_center])
        self.center_point = np.array((x ,y))
        radii = []
        theta = []
        for point in self.fish_center:
            pos = np.array(point)-self.center_point
            radii.append(np.linalg.norm(pos))
            

        rospy.loginfo(self.board_center)
        rospy.loginfo(self.fish_center)


    def mouse_press(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.board_center[self.calibration_index] = (x,y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.fish_center[self.calibration_index] = (x,y)
    
    def callback(self, image):
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        if not self.buffer_full:
            self.add_image(image)
        if self.calibrated:
            pass

def main(args):
  rospy.init_node('fish_detective')
  hff = HandFishFinder()
  hff.calibrate()
  cv2.destroyAllWindows()
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)