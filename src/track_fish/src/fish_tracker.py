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
DEQUE_SIZE = 2
MAX_ANGLE_ERROR = 0.001
ANGLE_STEP = 0.01
MOUTH_ERROR = 5.0



class FishTracker(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.skew_matrix = np.float32(calibration_data.skew_matrix)
        self.fish_locales = calibration_data.fishes
        # self.fish_locales.sort(key=lambda loc: sum(loc))
        # self.fish_locales = np.matrix(self.fish_locales)
        self.fish_holes = [FishHole() for i in range(len(self.fish_locales))]
        self.board_center = Point(*calibration_data.board_center)
        self.crop_center = Point(*calibration_data.crop_center)
        self.crop_radius = calibration_data.crop_radius
        self.openings = [Point(*opening) for opening in calibration_data.openings]
        self.theta = deque()
        self.t = deque()
        self.rate = 0.0
        self.offset = 0
        self.zeroed = False


    def crop_img(self, image):
        img = cv2.warpPerspective(image, self.skew_matrix, (700, 700))
        y = -self.crop_center.y
        x = self.crop_center.x
        r = self.crop_radius
        return img[y-r:y+r, x-r:x+r]


    def get_angle(self, t):
        return (self.rate)*t+self.offset

    
    def append(self, l, x):
        if len(l) < DEQUE_SIZE:
            l.append(x)
        else:
            l.popleft()
            self.append(l,x)


    def _init_ros(self):
        # self.pub_hough = rospy.Publisher("hough_image",Image, queue_size=2)
        self.pub_results = rospy.Publisher("fish_tracker",Image, queue_size=2)
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.find_holes)
        self.tracker_sub = rospy.Subscriber("/cv_camera/image_raw", Image, self.track_holes, queue_size = 1, buff_size=2**24)

    
    def give_fish(self):
        now = rospy.Time.now().to_time()
        t = now + 2
        angle = self.get_angle(t)
        fishes = self.rotate(self.fish_locales, angle)



    def track_holes(self, image):
        rospy.loginfo('Tracking Fish')
        t = image.header.stamp.to_time()
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        img = self.crop_img(image)
        angle = self.get_angle(t)
        fishes = self.rotate(self.fish_locales, angle)
        fish_found = 0
        for i in range(len(fishes)):
            if self.fish_holes[i].is_fish():
                location = Point(*fishes[i])+self.board_center
                if self.mouth_open(location):
                    cv2.circle(img,location.to_image(), 30,(0,255,0),2)
                    fish_found += 1
                    rospy.loginfo('Fish At {}'.format(location))
                else:
                    cv2.circle(img,location.to_image(), 30,(0,0,255),2)
                # rospy.loginfo('Fish At {}'.format(location))
        if __name__ == "__main__":
            self.pub_results.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        else:
            return img


    def mouth_open(self, location):
        for mouth in self.openings:
            err = location - mouth
            if err.magnitude() < MOUTH_ERROR:
                return True
        return False


    def isolate_board(self):
        pass

    def zero(self, locations):
        est = 0
        angle, results = self.get_angle_offset(self.fish_locales, locations, est, denom=0.5)
        self.rate = 0
        self.offset = angle
        self.zeroed = True


    def getCircles(self, img, rad):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,70,param1=10,
              param2=40, minRadius=25, maxRadius=35)

        circles = np.uint16(np.around(circles))
        
        rects = []
        for i in circles[0,:]: 
                 
            fish = Point(int(i[0]), int(i[1]), from_image=True)           
            local = fish-self.board_center      
            rects.append(local.get_tuple())
        return rects


    def find_holes(self, image):
        t = image.header.stamp.to_time()
        print 'Finding Holes'
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        img = self.crop_img(image)
        circles = self.getCircles(img, 30)
        locations = [c for c in circles]
        est = self.get_angle(t)
        if self.zeroed:
            angle, results = self.get_angle_offset(self.fish_locales, locations, est, denom=8)
            if results == None:
                rospy.loginfo('No angle found trying to reset')
                angle, results = self.get_angle_offset(self.fish_locales, locations, est, denom=0.5)
            self.calibrate(angle, t)
        else:
            print 'Zeroing'
            self.zero(locations)



    def rotate(self, l, theta):
        r = np.matrix([[np.cos(theta), np.sin(theta)],
                          [-np.sin(theta), np.cos(theta)]])
        x = np.matrix(l)
        rotated = x*r
        return rotated.tolist()


    def get_angle_offset(self, ref, comp, est, denom=0.5):
        angles = np.linspace(est-np.pi/denom, est+np.pi/denom, num=1000).tolist()
        # controller = PID(0.5, 0.01, 0.0, [-np.pi/4, np.pi/4])
        # aligned = False
        total_err = []
        results = []
        for angle in angles:
            rotated = self.rotate(ref, angle)
            res, err, matching = self.compare_lists(rotated, comp)
            if matching:
                total_err.append(err)
                results.append(res)
            else:
                angles.remove(angle)
        if len(total_err) < 1:
            rospy.logerr('No angle found!')
            return None, None
            # return self.get_angle_offset(ref, comp, est, denom=0.5)   
        indx = total_err.index(min(total_err))
        # print min(total_err)
        return angles[indx], results[indx]
        
   

    def compare_lists(self, ref, comp):
        results = dict()
        matching = True
        for item in comp:
            total_err = []
            for i in range(len(ref)):
                err = np.sqrt((ref[i][0]-item[0])**2 + (ref[i][1]-item[1])**2)
                total_err.append(err)
                # print 'Error of {}'.format(err)
            index = total_err.index(min(total_err))
            # print 'smallest error at index {}'.format(index)
            if index not in results.keys():
                # print 'placing item'
                results[index] = (total_err[index], item)
            else:
                matching = False
            
        total_err = []
        for key in results.keys():
            total_err.append(results[key][0])
        return results, np.mean(total_err), matching


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
            if len(rates) >= 2:
                rate = np.mean(rates)
            else:
                rate = rates[0]
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
