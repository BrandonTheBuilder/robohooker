#!/usr/bin/env python

# System imports
import sys, os
import time
import numpy as np
import copy
import imutils
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import calibration_data

# opencv imports
import cv2
import cv2.cv as cv

# ROS Imports 
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

# my own imports
from point import Point
CALIBRATION_LENGTH = 120

class Calibrator(object):
    def __init__ (self):
        self.bridge = CvBridge()
        self.calibrated = False
        self.calibrating = False
        self.ready_for_cams = False

    def _init_ros(self):
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.auto_calibrate, queue_size=1)


    def load_calibration(self):
        self.center_point = Point(*calibration_data.center_point)
        self.board_center = Point(*calibration_data.board_center)
        self.board_radius = calibration_data.board_radius
        self.fish = [Point(*a) for a in calibration_data.fish]
        self.cams = [Point(*c) for c in calibration_data.cams]
        self.base_cams = [Point(*c) for c in calibration_data.base_cams]
        self.skew_matrix = np.float32(calibration_data.skew_matrix)
        self.recalibrate = True


    def auto_calibrate(self, image):
        if not self.calibrating and not self.ready_for_cams:
            self.calibrating = True

            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            if self.recalibrate:
                color_space = [([170, 100, 55], [255, 175, 125]), ([0, 200, 200], [100, 240, 240])] #(blue,yellow) #yello:([0, 200, 200], [100, 240, 240])
                ratio = 1
                blue_lower, blue_upper = color_space[0]
                thresh = self.threshhold(blue_lower, blue_upper, image)
                
                # yellow_lower, yellow_upper = color_space[1]
                # yellow_thresh = self.threshhold(yellow_lower, yellow_upper, image)
                # yellow_blobs = self.get_blobs(yellow_thresh)
                # yel_len = [len(a) for a in yellow_blobs]
                # indx = yel_len.index(max(yel_len))
                # yellow = yellow_blobs[indx]
                # yell_centroid = self.get_center(*[Point(*a, from_image=True) for a in yellow])

                blobs = self.get_blobs(thresh)
                lengths = [len(a) for a in blobs]
                try:
                    indx = lengths.index(max(lengths))
                    blob = blobs[indx]
                    xmax = Point(*max(blob, key=lambda p: p[0]), from_image=True)
                    xmin = Point(*min(blob, key=lambda p: p[0]), from_image=True)
                    ymax = Point(*max(blob, key=lambda p: p[1]), from_image=True)
                    ymin = Point(*min(blob, key=lambda p: p[1]), from_image=True)
                    # center = self.get_center(*[Point(*b, from_image=True) for b in blob])
                    self.skew_points = [xmax, xmin, ymax, ymin]
                    

                    center = self.get_center(*self.skew_points)
                    self.center_point = center
                except:
                    skew_points = [Point(self.board_radius, 0, from_polar=True),
                                   Point(self.board_radius, np.pi/2, from_polar=True),
                                   Point(self.board_radius, np.pi, from_polar=True),
                                   Point(self.board_radius, np.pi*3/2, from_polar=True)]
                    self.skew_points = [p + self.board_center for p in skew_points]
                    pass

                
                self.check_points(image)
                self.check_center(image)

                self.board_center = self.center_point
                loc_points = [p-self.center_point for p in self.skew_points]
                xmax = max(loc_points, key=lambda p: p.x)
                xmin = min(loc_points, key=lambda p: p.x)
                radii = [p.magnitude() for p in [xmin, xmax]]
                theta = [p.angle() for p in loc_points]
                r = np.mean(radii)
                self.board_radius = r
                skewed = [Point(r, t, from_polar=True)+self.center_point for t in theta]
                before = np.float32([p.to_image() for p in self.skew_points])
                after = np.float32([p.to_image() for p in skewed])
                # self.center_point = self.get_center(*skewed)
                self.skew_matrix = cv2.getPerspectiveTransform(before, after)
                
                img = self.crop_img(image)

                y,x,c = img.shape
                self.center_point = Point(float(x)/2, float(y)/2, from_image=True)
                self.check_center(img)
                self.get_fish(img)
            self.get_cams(image)
            self.calibrated = True
        elif self.ready_for_cams and not self.calibrating:
            self.calibrating = True
            image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            self.locate_cams(image)
            self.calibrated = True


    def check_points(self, image):
        cv2.namedWindow("Points")
        cv2.startWindowThread()
        cv2.setMouseCallback("Points", self._check_points)
        while True:
            img = copy.copy(image)
            for point in self.skew_points:
                cv2.circle(img, point.to_image(),3,(255,0,0),3)
            cv2.imshow('Points', img)
            k = cv2.waitKey(33)
            if k==10:
                if len(self.skew_points) == 4:
                    break
                else:
                    print 'Select only four points'
            elif k==-1:
                pass
            else:
                print k
                print 'Press Enter to continue..'
        cv2.destroyWindow("Points")

    def _check_points(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.skew_points) < 4:
                self.skew_points.append(Point(x,y, from_image=True))
            else:
                print "Select only four points, right click to remove one"

        if event == cv2.EVENT_RBUTTONDOWN:
            target = Point(x,y,from_image=True)
            for point in self.skew_points:
                if (target-point).magnitude() < 10:
                    self.skew_points.remove(point)
    
    def get_fish(self, image):
        cv2.namedWindow("Fish")
        cv2.startWindowThread()
        cv2.setMouseCallback("Fish", self._get_fish)
        img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,70,param1=10,
              param2=40, minRadius=15, maxRadius=35)
        try:
            circles = np.uint16(np.around(circles))
            self.fish = []
            for i in circles[0,:]: 
                fish = Point(int(i[0]), int(i[1]), from_image=True)     
                local = fish-self.center_point      
                self.fish.append(local)
        except:
            pass
        
        while True:
            img = copy.copy(image)
            if len(self.fish) is not 0:
                for fish in self.fish:
                    location = fish + self.center_point
                    cv2.circle(img, location.to_image(),20,(0,255,0),3)
            cv2.imshow('Fish', img)
            k = cv2.waitKey(33)
            if k==10:
                if len(self.fish) < 21:
                    print 'Please select all 21 fish'
                else:
                    break
            elif k==-1:
                pass
            else:
                print k
                print 'Press Enter to continue..'
        cv2.destroyWindow('Fish')


    def _get_fish(self, event, x, y, flags, param):
        if event ==cv2.EVENT_LBUTTONDOWN:
            if len(self.fish) < 21:
                fish = Point(x,y,from_image=True)
                self.fish.append(fish-self.center_point)
            else:
                print 'Press enter to contiue or remove a fish to replace'
            
        elif event == cv2.EVENT_RBUTTONDOWN:
            target = Point(x,y,from_image=True)-self.center_point
            for fish in self.fish:
                if (target-fish).magnitude() < 25:
                    self.fish.remove(fish)


    def get_cams(self, image):
        cv2.namedWindow("Screw")
        cv2.startWindowThread()
        cv2.setMouseCallback("Screw", self._get_screw)
        self.screw_location = Point(0,0)
        while True:
            img = copy.copy(image)
            cv2.circle(img, self.screw_location.to_image(), 3, (0,0,255), 3)
            cv2.imshow('Screw', img)
            k = cv2.waitKey(33)
            if k==10:
                # enter pressed
                break
            elif k==-1:
                pass
            else:
                print k
                print 'Press Enter to continue..'
        cv2.destroyWindow('Screw')
        arc = self.screw_location - self.board_center
        cam_angle = arc.angle()
        self.cam_angle = cam_angle
        if len(self.base_cams) != 0:
            self.cams = self.rotate(self.base_cams, cam_angle)
            # self.cams = [c for c in self.cams if c.y <= 0]
            self.locate_cams(image)
            


    def _get_screw(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.screw_location = Point(x,y, from_image=True)
            
    
    def locate_cams(self, image):
        cv2.namedWindow("Cams")
        cv2.startWindowThread()
        cv2.setMouseCallback("Cams", self._locate_cams)
        img_cropped = self.crop_img(image)
        while True:
            img = copy.copy(img_cropped)
            for point in self.cams:
                location = point+self.center_point
                cv2.circle(img, location.to_image(),3,(255,0,0),3)
            cv2.imshow('Cams', img)
            k = cv2.waitKey(33)
            if k==10:
                break
                    
            elif k==-1:
                pass
            else:
                print k
                print 'Press Enter to continue..'
        cv2.destroyWindow("Cams")
        # self.base_cams = self.rotate(self.cams, -self.cam_angle)
        



    def _locate_cams(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if len(self.cams) < 8:
                self.cams.append(Point(x,y, from_image=True)-self.center_point)
            else:
                print "Select only four points, right click to remove one"

        if event == cv2.EVENT_RBUTTONDOWN:
            target = Point(x,y,from_image=True)
            for point in self.cams:
                if (target-(point+self.center_point)).magnitude() < 10:
                    self.cams.remove(point)

    def rotate(self, l, theta):
        a = [p.get_tuple() for p in l]
        r = np.matrix([[np.cos(theta), np.sin(theta)],
                          [-np.sin(theta), np.cos(theta)]])
        x = np.matrix(a)
        rotated = x*r
        return [Point(*p) for p in rotated.tolist()]

    def threshhold(self, lower, upper, image):
        # create NumPy arrays from the boundaries
        lower = np.array(lower, dtype = "uint8")
        upper = np.array(upper, dtype = "uint8")
        # find the colors within the specified boundaries and apply
        # the mask
        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)
        # convert the resized image to grayscale, blur it slightly,
        # and threshold it
        gray = cv2.cvtColor(output, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]
        return thresh


    def check_center(self, image):
        cv2.namedWindow("Center")
        cv2.startWindowThread()
        cv2.setMouseCallback("Center", self._check_center)
        while True:
            center = self.center_point
            img = copy.copy(image)
            if center is not 0:
                cv2.circle(img, center.to_image(),3,(255,0,0),3)
            cv2.imshow('Center', img)
            k = cv2.waitKey(33)
            if k==10:
                # enter pressed
                break
            elif k==-1:
                pass
            else:
                print k
                print 'Press Enter to continue..'
        cv2.destroyWindow('Center')


    def _check_center(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.center_point = Point(x,y, from_image=True)


    def get_center(self, *args):
        x = 0 
        y = 0 
        num = 0
        for arg in args:
            x += arg.x
            y += arg.y
            num += 1
        return Point(float(x)/num,float(y)/num)


    def get_blobs(self, img):
        blob_points = set()
        blobs = []
        blob_index = 0
        xmax = img.shape[1]
        ymax = img.shape[0]
        for y in range(ymax):
            for x in range(xmax):
                if (x,y) not in blob_points:
                    val = img[y,x]
                    if val != 0:
                        blob = set()
                        blob.add((x,y))
                        checked = set()
                        checked.add((x,y))
                        to_check = set()
                        i = x
                        j = y
                        neighbors = [(i-1,j),(i,j-1),(i-1,j-1),(i+1,j),(i,j+1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]
                        neighbors = [n for n in neighbors if n[0] < xmax and n[1] < ymax and n[0] >= 0 and n[1] >= 0]
                        to_check.update([n for n in neighbors if n not in checked])
                        while len(to_check) > 0:
                            point = to_check.pop()
                            i,j = point
                            checked.add(point)
                            # print checked
                            val = img[point[1], point[0]]
                            if val != 0:
                                blob.add(point)
                                neighbors = [(i-1,j),(i,j-1),(i-1,j-1),(i+1,j),(i,j+1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]
                                neighbors = [n for n in neighbors if n[0] < xmax and n[1] < ymax and n[0] >= 0 and n[1] >= 0]
                                to_check.update([n for n in neighbors if n not in checked])
                        blobs.append(blob)
                        blob_points.update(blob)
                        blob_index += 1
        return blobs


    def crop_img(self, image):
        img = cv2.warpPerspective(image, self.skew_matrix, (700, 700))
        y = -self.board_center.y
        x = self.board_center.x
        r = self.board_radius
        return image[y-r:y+r, x-r:x+r]

    def write(self, data):
        # center_point
        data.write('center_point = {} \n'.format(self.center_point.get_tuple()))
        # board_center
        data.write('board_center = {} \n'.format(self.board_center.get_tuple()))
        # board_radius
        data.write('board_radius = {} \n'.format(self.board_radius))
        data.write('board_edges = {} \n'.format([e.get_tuple() for e in self.skew_points]))
        # fish
        data.write('fish = {} \n'.format([f.get_tuple() for f in self.fish]))
        # cams
        data.write('base_cams = {} \n'.format([c.get_tuple() for c in self.base_cams]))
        data.write('cams = {} \n'.format([c.get_tuple() for c in self.cams if c.y <= 0]))
        # skew_matrix
        data.write('skew_matrix = {} \n'.format(str(self.skew_matrix.tolist())))



def main(args):
    rospy.init_node('track_fish')
    c = Calibrator()
    c._init_ros()
    c.load_calibration()
    while not c.calibrated:
        pass
    if len(c.cams) == 0:
        c.ready_for_cams = True
        c.calibrated = False
        c.calibrating = False
    while not c.calibrated:
        pass
    data_file = os.path.join(os.path.dirname(__file__), 'calibration_data.py')
    calibration_data = open(data_file, 'w')
    c.write(calibration_data)


if __name__ == '__main__':
    main(sys.argv)