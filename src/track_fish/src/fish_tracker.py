#!/usr/bin/env python
# System imports
import sys, os
import numpy as np
from collections import deque

# opencv imports
import cv2
import cv2.cv as cv

# ROS Imports 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# my own imports
from point import Point
from fish_hole import FishHole
from pid import PID
import calibration_data

# Constants
DEQUE_SIZE = 30
MAX_ANGLE_ERROR = 400
ANGLE_STEP = 0.01



class FishTracker(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.skew_matrix = np.float32(calibration_data.skew_matrix)
        self.fish_locales = calibration_data.fishes
        # self.fish_locales.sort(key=lambda loc: sum(loc))
        # self.fish_locales = np.matrix(self.fish_locales)
        self.fish_holes = [FishHole() for i in range(len(self.fish_locales))]
        self.board_center = Point(*calibration_data.board_center)
        self.theta = deque()
        self.t = deque()
        self.rate = -0.95
        self.offset = 0
        edges = []
        for edge in calibration_data.board_edges:
            e = Point(*edge)
            p = e-self.board_center
            edges.append(e.magnitude())
        self.r = np.mean(edges)

    def crop_img(self, img):
        y = -self.board_center.y
        x = self.board_center.x
        r = self.r
        return img[y-r:y+r, x-r:x+r]
        



    def get_angle(self, t):
        return self.rate*t+self.offset

    
    def append(self, l, x):
        if len(l) < DEQUE_SIZE:
            l.append(x)
        else:
            l.popleft()
            self.append(l,x)


    def _init_ros(self):
        self.pub_hough = rospy.Publisher("hough_image",Image, queue_size=2)
        self.pub_results = rospy.Publisher("fish_tracker",Image, queue_size=2)
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.find_holes)


    def isolate_board(self):
        pass


    def getCircles(self, img, rad):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,70,param1=10,
              param2=40, minRadius=25, maxRadius=35)

        circles = np.uint16(np.around(circles))
        
        rects = []
        for i in circles[0,:]:          
            fish = Point(i[0], -i[1])           
            cv2.circle(img,fish.to_image(),30,(0,255,0),2)            
            local = fish-self.board_center      
            rects.append(((i[0], i[1]), img[i[1]-rad:i[1]+rad, i[0]-rad:i[0]+rad]))
        self.pub_hough.publish(self.bridge.cv2_to_imgmsg(img, "8UC1"))
        return rects


    def find_holes(self, image):
        t = image.header.stamp.to_time()
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        img = cv2.warpPerspective(image, self.skew_matrix, (700, 700))
        circles = self.getCircles(img, 30)
        locations = [c[0] for c in circles]
        est = self.get_angle(t)
        rospy.loginfo('looking for angle')
        angle, results = self.get_angle_offset(self.fish_locales, locations, est)
        rospy.loginfo('Found angle! {}'.format(angle))
        self.calibrate(angle, t)
        for key in results.keys():
            index = locations.index(results[key][1])
            self.fish_holes[index].fishy_calibration(circles[index][1])
        fishes = self.rotate(self.fish_locales, self.get_angle(t))
        for i in range(len(fishes)):
            location = Point(*fishes[i])+self.board_center
            x,y = location.to_image()
            rect = img[y-30:y+10, x-30:x+30]
            self.fish_holes[i].fishy_calibration(rect)

            if self.fish_holes[i].is_fish():
                cv2.circle(img,location.to_image(), 30,(0,255,0),2)
            else:
                cv2.circle(img,location.to_image(), 30,(255,0,0),2)


        
        self.pub_results.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))


    def rotate(self, l, theta):
        r = np.matrix([[np.cos(theta), np.sin(theta)],
                          [-np.sin(theta), np.cos(theta)]])
        x = np.matrix(l)
        rotated = x*r
        return rotated.tolist()


    def get_angle_offset(self, ref, comp, est):
        controller = PID(0.01, 0.0, 0.0, [-np.pi/4, np.pi/4])
        aligned = False
        total_err = []
        while not aligned:
            ref = self.rotate(ref, est)
            res, err = self.compare_lists(ref, comp)
            rospy.loginfo('Average Error of {}'.format(err))
            if err > MAX_ANGLE_ERROR:
                est = controller.update(err)
                aligned = True
            else:
                aligned = True        
        return est, res
        
   

    def compare_lists(self, ref, comp):
        results = dict()
        matching = True
        for item in comp:
            total_err = []
            for i in range(len(ref)):
                err = abs(ref[i][0]-item[0]) + abs(ref[i][1]-item[1])
                total_err.append(err)
                # print 'Error of {}'.format(err)
            index = total_err.index(min(total_err))
            # print 'smallest error at index {}'.format(index)
            if index not in results.keys():
                # print 'placing item'
                results[index] = (total_err[index], item)
            else:
                # print 'Sorry no match'
                matching = False
        total_err = []
        for key in results.keys():
            total_err.append(results[key][0])
        return results, np.mean(total_err)


    def calibrate(self, angle, t):
        """
        """
        self.append(self.theta, angle)
        self.append(self.t, t)
        if len(self.t) < 2:
            print 'Calibrating'
            self.rate = 0
            self.offset = angle
        else:
            rates = []
            for i in range(len(self.t)):
                if i >= 1:
                    dTheta = self.theta[i]-self.theta[i-1]
                    dt = self.t[i]-self.t[i-1]
                    rates.append(dTheta/dt)
            rate = np.mean(rates)
            bs = []
            for i in range(0, len(self.t)-1):
                bs.append(self.theta[i] - rate*self.t[i])
            b = np.mean(bs)
            self.rate = rate
            self.offset = b
            self.calibrated = True

def main(args):
    rospy.init_node('fish_tracker')
    ft = FishTracker()
    ft._init_ros()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
