#!/usr/bin/env python
import sys
import rospy
import cv2
import cv2.cv as cv
import numpy as np
import imutils
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


from collections import deque

# Yellow Lower aeb634
# 64, 71.4, 71.4
# 62, 62.5, 91
redLow = np.uint8([[[0,255,0 ]]])
redHigh = np.uint8([[[0,255,0 ]]])
yellowLower = cv2.cvtColor(green,cv2.COLOR_BGR2HSV)
yellowUpper = (400, int((70.5/100)*255), int((255.4/100)*255))
whiteLower =(0,225,50)
whiteUpper =(180,255,255)

blueLower =(90,20,0)
blueUpper = (140,255,255)
imsize=600

class BlobTracker(object):
    def __init__(self):
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher("image_topic_2",Image, queue_size=2)
        self.image_sub = rospy.Subscriber("/cv_camera/image_raw",Image,self.callback)
        self.board_frame = [None, None, None, None]

    def resize(self, image, x1, x2, y1, y2):
        # frame = imutils.resize(image, width=imsize)
        return image[y1:y2, x1:x2]
    
    def callback(self, image):
        frame = self.bridge.imgmsg_to_cv2(image, "bgr8")
        if None in self.board_frame:
            self.size_board(frame)
        # Resize image to match the board size.
        # frame = self.resize(frame, *self.board_frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        mask_w = cv2.inRange(hsv, yellowLower, yellowUpper)

        #mask_w = cv2.erode(mask_w, None, iterations=2)
        mask_w = cv2.dilate(mask_w, None, iterations=2)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask_w.copy(),
                                cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        fishcount=0
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            for c in range(0,len(cnts)):
                # find the largest contour in the mask, then use
                # it to compute the minimum enclosing circle and
                # centroid
                #c = max(cnts, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(cnts[c])
                M = cv2.moments(cnts[c])
                center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
                # only proceed if the radius meets a minimum size (default was 10)
                #print ("Radius:",radius)
                if radius >25 and radius <35:
                        fishcount=fishcount+1
                        # draw the circle and centroid on the frame,
                        # then update the list of tracked points
                        #first color is 0,255,255
                        #print "Green"
                        cv2.circle(frame, (int(x), int(y)), int(radius),(0, 0, 0), 2)
                #fishtext=str(fishcount)
                #cv2.putText(frame,fishtext,(10,20),font,1,(255,255,255),1)
        rospy.loginfo('Found {} fish!'.format(fishcount))
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

                


    def size_board(self, frame):
        frame = self.resize(frame, 0, imsize, 0 , imsize)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask_b = cv2.inRange(hsv, blueLower, blueUpper)
        #cv2.imshow("Blue1", mask_b)
        mask_b = cv2.erode(mask_b, None, iterations=1)
        #cv2.imshow("Blue2", mask_b)
        mask_b = cv2.dilate(mask_b, None, iterations=8)
        #cv2.imshow("Blue3", mask_b)
        # find contours in the mask and initialize the current
        # (x, y) center of the ball
        cnts = cv2.findContours(mask_b.copy(), cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE)[-2]
        center = None
        # only proceed if at least one contour was found
        if len(cnts) > 0:
            # find the largest contour in the mask, then use
            # it to compute the minimum enclosing circle and
            # centroid
            c = max(cnts, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            M = cv2.moments(c)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            
            # only proceed if the radius meets a minimum size (default was 10)
            if radius > 20:
                # draw the circle and centroid on the frame,
                # then update the list of tracked points
                #first color is 0,255,255
                            #print "Blue!"
                x=int(x)
                y=int(y)
                radius=int(radius)
                cv2.circle(mask_b, (x, y), radius,(255, 0, 0), 2)
                #cv2.imshow("Blue3",mask_b)
                x1=x-radius
                x2=x+(radius)
                y1=y-radius
                y2=y+radius
                self.board_frame = [x1, x2, y1, y2]
                rospy.loginfo('Board frame of ({},{}),({},{}) found'.format(*self.board_frame))
                return True
            rospy.logerr('Board sizing failed, radius of {} is less than 20'.format(radius))
            return False
        rospy.logerr('Board Sizing failed, no contours found')
        return False




def main(args):
    rospy.init_node('fish_detective')
    bt = BlobTracker()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)