#!/usr/bin/env python

# System imports
import sys, os
import numpy as np
import copy

# opencv imports
import cv2
import cv2.cv as cv

# ROS Imports 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# my own imports
from point import Point

class Calibrator(object):
    def __init__ (self):
        self.bridge = CvBridge()
        self.calibrating = False
        self.calibrated = False

    def _init_ros(self):
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.calibrate)

    def find_skew(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print 'Left Click at {}'.format((x,y))
            self.skew_points[self.skew_index] = Point(x,y, from_image=True)
            if self.skew_index < 4:
                self.skew_index += 1
            else:
                print 'Press enter to continue...'
            
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.skew_points[self.skew_index] is 0 and self.skew_index > 0:
                self.skew_index -= 1
            self.skew_points[self.skew_index] = 0
            

    
    def find_fish(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print 'Left Click at {}'.format((x,y))
            self.fishes[self.fish_index] = Point(x,y, from_image=True)
            if self.fish_index < 23:
                self.fish_index += 1
            else:
                print 'Press enter to continue...'
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.fishes[self.fish_index] is 0 and self.fish_index > 0:
                self.fish_index -= 1
            self.fishes[self.fish_index] = 0
            

    def calibrate(self, image):
        if self.calibrating:
            pass
        elif self.calibrated:
            pass
        else:
            self.calibrating = True
            self.skew_index = 0
            self.skew_points = [0]*5
            cv2.namedWindow("Calibration")
            cv2.startWindowThread()
            cv2.setMouseCallback("Calibration", self.find_skew)
            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            while self.skew_index <= 4:
                img = copy.copy(image)
                for point in self.skew_points:
                    if point is not 0:
                        cv2.circle(img, point.to_image(),3,(255,0,0),3)
                cv2.imshow('Calibration', img)
                k = cv2.waitKey(33)
                if k==10:
                    # enter pressed
                    if self.skew_points[4] is not 0:
                        break
                    else:
                        print 'Please select center and four edge points'
                elif k==-1:
                    pass
                else:
                    print k
                    print 'Press Enter to continue..'
            self.center_point = self.skew_points[0]
            glob_points = [p for p in self.skew_points[1:5]]
            loc_points = [p-self.center_point for p in glob_points]
            radii = [p.magnitude() for p in loc_points]
            theta = [p.angle() for p in loc_points]
            r = np.mean(radii)
            skewed = [Point(r, t, from_polar=True)+self.center_point for t in theta]
            before = np.float32([p.to_image() for p in glob_points])
            after = np.float32([p.to_image() for p in skewed])
            self.skew_matrix = cv2.getPerspectiveTransform(before, after)
            # x_offset = r-self.center_point.x
            # translate_matrix = np.float32([[ 1 , 0 , x_offset],[ 0 , 1 , 0]])
            self.fish_index = 0
            self.fishes = [0]*24
            cv2.setMouseCallback("Calibration", self.find_fish)
            img = cv2.warpPerspective(image, self.skew_matrix, (700, 700))
            # img = cv2.warpAffine(img,translate_matrix,(int(r*2),int(r*2)))
            while True:
                image = copy.copy(img)
                for i in range(len(self.fishes)):
                    if self.fishes[i] is not 0:
                        if i <= 2:
                            cv2.circle(image, self.fishes[i].to_image(), 3, (255,0,0), 3)
                        else:
                            cv2.circle(image, self.fishes[i].to_image(),30,(255,0,0),3)
                cv2.imshow('Calibration', image)
                k = cv2.waitKey(33)
                if k==10:
                    # enter pressed
                    if self.skew_points[1] is not 0:
                        break
                    else:
                        print 'Please select center and all fish'
                elif k==-1:
                    pass
                else:
                    print k
                    print 'Press Enter to continue..'
            for i in range(1, len(self.fishes)):
                if self.fishes[i] is not 0:
                    self.fishes[i] = self.fishes[i]-self.fishes[0]
            cv2.destroyWindow('Calibration')
            self.calibrated = True

def main(args):
    rospy.init_node('track_fish')
    c = Calibrator()
    c._init_ros()
    while not c.calibrated:
        pass
    data_file = os.path.join(os.path.dirname(__file__), 'calibration_data.py')
    calibration_data = open(data_file, 'w')
    calibration_data.write('board_center = {} \n'.format(str(c.fishes[0])))
    calibration_data.write('board_edges = {} \n'.format([str(c.fishes[1]), str(c.fishes[2])]))
    calibration_data.write('skew_matrix = {} \n'.format(str(c.skew_matrix.tolist())))
    calibration_data.write('fishes = {} \n'.format([f.get_tuple() for f in c.fishes[3:]]))

if __name__ == '__main__':
    main(sys.argv)