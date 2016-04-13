#!/usr/bin/env python
import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import time

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from scipy.optimize import leastsq
from point import Point
BUFFER_SIZE = 4
TIME_STEP = 0.1

class HandFishFinder(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=2)
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
        self.calibration_buffer = []
        self.buffer_full = False
        self.calibrated = False


    def add_image(self, image, t):
        
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
        while not self.buffer_full:
            time.sleep(0.5)
        cv2.namedWindow("Calibration")
        cv2.startWindowThread()
        cv2.setMouseCallback("Calibration", self.mouse_press)
        self.calibration_index = 0
        self.board_center = [(0,0)]*BUFFER_SIZE
        self.fish_center = [(0,0)]*BUFFER_SIZE
        t_steps = []
        for img in self.calibration_buffer:
            self.calibration_image = img[1]
            t_steps.append(img[0])
            self.calibration_phase = 'board'
            cv2.imshow('Calibration', img[1])
            k = cv2.waitKey(0)
            self.calibration_index += 1
        cv2.destroyWindow('Calibration')
        # Now that we have some data let's use it
        x = sum([point.x for point in self.board_center])
        y = sum([point.y for point in self.board_center])
        
        self.center_point = Point(x,y)
        self._calibrate(self.center_point, self.fish_center, t_steps)
        self.calibrated = True

    def _calibrate(self, board_center, fish_centers, t):
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
        self.find_angle = lambda t_step: (np.pi + np.pi*np.sin(freq*t_step+phase)) 
        self.calibrated = True
        

    def angle(self, x1,x2):
        return np.arctan2(x2,x1)


    def mouse_press(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.board_center[self.calibration_index] = Point(x,-y)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.fish_center[self.calibration_index] = Point(x,-y)

    
    def callback(self, image):
        t = rospy.get_time()
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        if not self.buffer_full:
            self.add_image(image, t)
        if self.calibrated:
            theta = self.find_angle(t)
            rospy.loginfo('center {}'.format(self.center_point))
            fish_pos = Point(self.radius, theta, from_polar=True)
            glob_fish_pos = fish_pos+self.center_point
            cv2.circle(image,(int(self.center_point.x),int(-self.center_point.y)),10,(255,255,255),3)
            cv2.circle(image,(int(glob_fish_pos.x), int(-glob_fish_pos.y)), 30,(0,255,0),2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(image, "bgr8"))




def main(args):
  rospy.init_node('fish_detective')
  hff = HandFishFinder()
  hff.calibrate()
  cv2.destroyAllWindows()
  rospy.spin()

if __name__ == '__main__':
    main(sys.argv)