from point import Point

import numpy as np
import rospy



class FishTracker(object):
    def __init__(self):
        self.fish_holes = []
        

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
            glob_location = Point(i[0], i[1])
            local = glob_location-self.board_center
            rects.append(, img[i[1]-rad:i[1]+rad, i[0]-rad:i[0]+rad])
        return rects


    def find_holes(self, image):
        pass


    def check_point(self, fish_center, t):
        pass


