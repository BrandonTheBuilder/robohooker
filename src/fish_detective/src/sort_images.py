#!/usr/bin/env python
import rosbag

import sys
import cv2
import cv2.cv as cv
import numpy as np

from cv_bridge import CvBridge

#BAG_NAME = '../bags/2016-04-06-16-57-50.bag' #1437
BAG_NAME = '../training/bags/no_fish.bag'
CLOSE = '../training/close/'
OPEN = '../training/open/'
DISCARD = '../training/discard/'
NEGATIVE = '../training/negative/'

class ImageSorter(object):
    def __init__(self):
        bridge = CvBridge()
        bag = rosbag.Bag(BAG_NAME)
        self.imgs = []
        for topic, msg, t in bag.read_messages(topics=['/cv_camera/image_raw/']):
            self.imgs.append(bridge.imgmsg_to_cv2(msg, "bgr8"))
        self.num_imgs = len(self.imgs)
        

    def find_no_fish(self):
        img_num = 0
        for img in self.imgs[:]:
            rect_num = 0
            rects = self.getCircles(img, 70)
            for rect in rects:
                f_name = NEGATIVE+str(str(img_num)+'_'+str(rect_num)+'.png')
                cv2.imwrite(f_name, rect)
                rect_num+=1
            img_num+=1


    def getCircles(self, img, rad):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,70,param1=10,
              param2=40, minRadius=25, maxRadius=35)
        circles = np.uint16(np.around(circles))
        rects = []
        for i in circles[0,:]:
            rects.append(img[i[1]-rad:i[1]+rad, i[0]-rad:i[0]+rad])
        return rects

    # 1437 
    def processImage(self):
        img_num = 0
        window = 'Up Open, Down Closed, Right Empty, Left Discard'
        cv2.namedWindow(window)
        for img in self.imgs[:]:
            print self.num_imgs - img_num
            rect_num = 0
            rects = self.getCircles(img, 35)
            length = len(rects)
            for rect in rects:
                failed = False
                f_name = str(str(img_num)+'_'+str(rect_num)+'.png')
                print length -rect_num
                
                try:
                    cv2.imshow(window, rect)
                    k = cv2.waitKey(0)
                except:
                    failed = True
                if k==65362:
                    # Up pressed
                    f_name = OPEN+f_name
                    
                elif k==65364:
                    # Down Pressed
                    f_name = CLOSE+f_name
                    
                elif k==65361:
                    # Left pressed
                    f_name = DISCARD+f_name
                    
                elif k==65363:
                    # Right pressed
                    f_name = NEGATIVE+f_name
                else:
                    print k
                    failed = True
                if not failed:
                    cv2.imwrite(f_name, rect)       
                rect_num+=1
                cv2.destroyWindow(window)
            img_num += 1


    def check_keys(self, img):
        while(1):
            cv2.imshow('img',img)
            k = cv2.waitKey(33)
            if k==1048603:    # Esc key to stop
                break
            elif k==-1:  # normally -1 returned,so don't print it
                continue
            else:
                print k # else print its value

if __name__ == '__main__':
    s = ImageSorter()
    s.find_no_fish()
    import IPython; IPython.embed()
    


