from point import Point
from collections import deque
import numpy as np
import cv2
DEQUE_SIZE = 10
CASCADE_CLASSIFIER = '../../fish_detective/training/harr_fish/cascade.xml'
whiteLower =(0,225,50)
whiteUpper =(180,255,255)

HARR_CUTOFF = 1.0
# ANGLE_TOLERANCE = 0.01
LOCATION_TOLERANCE = 100

class FishHole(object):
    """docstring for FishHole"""
    def __init__(self):
        self.fish = True
        # super(FishHole, self).__init__()
        # self.find_fish = cv2.CascadeClassifier(CASCADE_CLASSIFIER)
        # self.fishiness = deque()


    # def append(self, l, x):
    #     if len(l) < DEQUE_SIZE:
    #         l.append(x)
    #     else:
    #         l.popleft()
    #         self.append(l,x)


    # def fishy_calibration(self, img):
    #     # cv2.namedWindow("Fish")
    #     # cv2.startWindowThread()
    #     # cv2.imshow('Fish', img)
    #     # cv2.waitKey(0)
    #     # fishes = self.find_fish.detectMultiScale(img, 1.01, 1, 
    #                                              # minSize = (25,25), 
    #                                              # maxSize=(35,35))
    #     self.append(self.fishiness, 2)
    #     # hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #     # mask_w = cv2.inRange(hsv, whiteLower, whiteUpper)
    #     # # cv2.imshow("White1", mask_w)
    #     # #mask_w = cv2.erode(mask_w, None, iterations=2)
    #     # # cv2.imshow("White2", mask_w)
    #     # mask_w = cv2.dilate(mask_w, None, iterations=2)
    #     # # cv2.imshow("White3", mask_w)
    #     # # find contours in the mask and initialize the current
    #     # # (x, y) center of the ball
    #     # cnts = cv2.findContours(mask_w.copy(),
    #     #                         cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
    #     # center = None
    #     # fishcount=0
    #     # print len(cnts)
    #     # self.append(self.fishiness, len(cnts))
    #     # only proceed if at least one contour was found
    #     # if len(cnts) > 0:
    #     #     for c in range(0,len(cnts)):
    #     #         # find the largest contour in the mask, then use
    #     #         # it to compute the minimum enclosing circle and
    #     #         # centroid
    #     #         #c = max(cnts, key=cv2.contourArea)
    #     #         ((x, y), radius) = cv2.minEnclosingCircle(cnts[c])
    #     #         M = cv2.moments(cnts[c])
    #     #         center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
    #     #         # only proceed if the radius meets a minimum size (default was 10)
    #     #         #print ("Radius:",radius)
    #     #         if radius > 9 and radius <13:
    #     #                 fishcount=fishcount+1
    #     #                 # draw the circle and centroid on the frame,
    #     #                 # then update the list of tracked points
    #     #                 #first color is 0,255,255
    #     #                 #print "Green"
    #     #                 cv2.circle(frame, (int(x), int(y)), int(radius),(0, 0, 0), 2)
    #     #         #fishtext=str(fishcount)
    #     #         #cv2.putText(frame,fishtext,(10,20),font,1,(255,255,255),1)
    #     #         print("Fish COunt: ",fishcount)


    def is_fish(self):
        return self.fish
        # fishy_level = np.mean(self.fishiness)
        # if fishy_level < HARR_CUTOFF:
        #     return False
        # else:
        #     return True


    
        