#!/usr/bin/env python

# System imports
import sys, os
import time
import numpy as np
import copy
import imutils
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from shapefinder import ShapeDetector

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
        self.calibrating = True
        self.calibrated = False
        self.images= []
        self.open_index = 0
        self.openings = dict()

    def _init_ros(self):
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.calibrate)
        rospy.Subscriber("/cv_camera/image_raw", Image, self.find_cams)

    
    def get_detector(self):
        # Setup SimpleBlobDetector parameters.
        params = cv2.SimpleBlobDetector_Params()
         
        # Change thresholds
        # params.filterByColor = True
        # params.blobColor = 255
        params.minThreshold = 100;
        params.maxThreshold = 255;
         
        # Filter by Area.
        params.filterByArea = True
        params.minArea = 2000
         
        # Filter by Circularity
        params.filterByCircularity = False
        # params.minCircularity = 0.1
         
        # Filter by Convexity
        params.filterByConvexity = False
        # params.minConvexity = 0.87
         
        # Filter by Inertia
        params.filterByInertia = False
        # params.minInertiaRatio = 0.01
         
        # Create a detector with the parameters
        ver = (cv2.__version__).split('.')
        if int(ver[0]) < 3 :
            detector = cv2.SimpleBlobDetector(params)
        else : 
            detector = cv2.SimpleBlobDetector_create(params)
        return detector


    def auto_calibrate(self, image):
        color_space = [([125, 100, 45], [255, 200, 150]), ([0, 200, 200], [100, 240, 240])] #(blue,yellow) #yello:([0, 200, 200], [100, 240, 240])
        ratio = 1
        blue_lower, blue_upper = color_space[0]
        thresh = self.threshhold(blue_lower, blue_upper, image)
        
        yellow_lower, yellow_upper = color_space[1]
        yellow_thresh = self.threshhold(yellow_lower, yellow_upper, image)
        yellow_blobs = self.get_blobs(yellow_thresh)
        yel_len = [len(a) for a in yellow_blobs]
        indx = yel_len.index(max(yel_len))
        yellow = yellow_blobs[indx]
        yell_centroid = self.get_center(*[Point(*a, from_image=True) for a in yellow])

        blobs = self.get_blobs(thresh)
        lengths = [len(a) for a in blobs]
        indx = lengths.index(max(lengths))
        blob = blobs[indx]

        xmax = Point(*max(blob, key=lambda p: p[0]), from_image=True)
        xmin = Point(*min(blob, key=lambda p: p[0]), from_image=True)
        ymax = Point(*max(blob, key=lambda p: p[1]), from_image=True)
        ymin = Point(*min(blob, key=lambda p: p[1]), from_image=True)
        # center = self.get_center(*[Point(*b, from_image=True) for b in blob])
        glob_points = [xmax, xmin, ymax, ymin]
        center = self.get_center(*glob_points)
        self.center_point = center
        self.check_center(image)
        loc_points = [p-self.center_point for p in glob_points]
        radii = [p.magnitude() for p in loc_points[:2]]
        theta = [p.angle() for p in loc_points]
        r = np.mean(radii)
        self.r = r
        skewed = [Point(r, t, from_polar=True)+self.center_point for t in theta]
        before = np.float32([p.to_image() for p in glob_points])
        after = np.float32([p.to_image() for p in skewed])
        self.center_point = self.get_center(*skewed)
        self.skew_matrix = cv2.getPerspectiveTransform(before, after)
        
        img = self.crop_img(image)

        y,x,c = img.shape
        self.center_point = Point(float(x)/2, float(y)/2, from_image=True)
        self.check_center(img)
        self.get_fish(img)


    def get_fish(self, image):
        cv2.namedWindow("Fish")
        cv2.startWindowThread()
        cv2.setMouseCallback("Fish", self._get_fish)
        img = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,70,param1=10,
              param2=40, minRadius=15, maxRadius=35)
        circles = np.uint16(np.around(circles))
        self.fish = []
        for i in circles[0,:]: 
            fish = Point(int(i[0]), int(i[1]), from_image=True)     
            local = fish-self.center_point      
            self.fish.append(local)
        while True:
            img = copy.copy(image)
            if len(self.fish) is not 0:
                for fish in self.fish:
                    location = fish + self.center_point
                    cv2.circle(img, location.to_image(),25,(0,255,0),3)
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
        if event == cv2.EVENT_LBUTTONDOWN:
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


    def find_cams(self, frame):
        if len(self.images) <= CALIBRATION_LENGTH:
            self.images.append(self.bridge.imgmsg_to_cv2(frame, "bgr8"))
        elif not self.calibrated:
            while self.calibrating:
                time.sleep(0.5)
            
            cv2.namedWindow("FindCams")
            cv2.startWindowThread()
            cv2.setMouseCallback("FindCams", self.find_open)
            for image in self.images:
                img = self.crop_img(image)
                for i in range(len(self.openings)):
                    if i in self.openings.keys():
                        cv2.circle(img, self.openings[i].to_image(), 3, (0,0,255), 3)
                cv2.imshow("FindCams", img)
                k = cv2.waitKey(0)
            cv2.destroyWindow("FindCams")
            self.calibrated = True
            for key in self.openings.keys():
                self.openings[key] = self.openings[key] - self.fishes[0]

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


    def find_open(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print 'Left Click at {}'.format((x,y))
            self.openings[self.open_index] = Point(x,y, from_image=True)
            self.open_index += 1
            
        elif event == cv2.EVENT_RBUTTONDOWN:
            if self.openings[self.open_index] is 0 and self.open_index > 0:
                self.open_index -= 1
            self.openings[self.open_index] = 0


    def crop_img(self, image):
            img = cv2.warpPerspective(image, self.skew_matrix, (700, 700))
            y = -self.center_point.y
            x = self.center_point.x
            r = self.r
            return image[y-r:y+r, x-r:x+r]
    

    def calibrate(self, image):
        image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        if self.calibrating:
            self.skew_index = 0
            self.skew_points = [0]*5
            cv2.namedWindow("Calibration")
            cv2.startWindowThread()
            cv2.setMouseCallback("Calibration", self.find_skew)
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
            self.r = r
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
                image = self.crop_img(image)
                for i in range(len(self.fishes)):
                    if self.fishes[i] is not 0:
                        if i <= 2:
                            cv2.circle(image, self.fishes[i].to_image(), 3, (255,0,0), 3)
                        else:
                            cv2.circle(image, self.fishes[i].to_image(),20,(255,0,0),3)

                cv2.imshow('Calibration', image)
                k = cv2.waitKey(33)
                if k==10:
                    # enter pressed
                    if self.fishes[23] is not 0:
                        break
                    else:
                        print 'Please select all fish'
                elif k==-1:
                    pass
                else:
                    print k
                    print 'Press Enter to continue..'
            for i in range(1, len(self.fishes)):
                if self.fishes[i] is not 0:
                    self.fishes[i] = self.fishes[i]-self.fishes[0]
            cv2.destroyWindow('Calibration')
            self.calibrating = False
        
            


def main(args):
    rospy.init_node('track_fish')
    c = Calibrator()
    c._init_ros()
    while not c.calibrated:
        pass
    data_file = os.path.join(os.path.dirname(__file__), 'calibration_data.py')
    calibration_data = open(data_file, 'w')
    calibration_data.write('board_center = {} \n'.format(c.fishes[0].get_tuple()))
    calibration_data.write('board_edges = {} \n'.format([c.fishes[1].get_tuple(), c.fishes[2].get_tuple()]))
    calibration_data.write('crop_center = {} \n'.format(c.center_point.get_tuple()))
    calibration_data.write('crop_radius = {} \n'.format(c.r))
    calibration_data.write('skew_matrix = {} \n'.format(str(c.skew_matrix.tolist())))
    calibration_data.write('fishes = {} \n'.format([f.get_tuple() for f in c.fishes[3:]]))
    calibration_data.write('openings = {} \n'.format([c.openings[key].get_tuple() for key in c.openings.keys()]))


if __name__ == '__main__':
    main(sys.argv)