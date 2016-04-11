#!/usr/bin/env python
import rosbag

import sys
import cv2
import cv2.cv as cv
import numpy as np

from cv_bridge import CvBridge

BAG_NAME = '../bags/2016-04-06-16-57-50.bag'
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
        

    def getCircles(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        circles = cv2.HoughCircles(img,cv.CV_HOUGH_GRADIENT,0.5,100,param1=2,
              param2=5, minRadius=25, maxRadius=40)
        circles = np.uint16(np.around(circles))
        rects = []
        for i in circles[0,:]:
            rects.append(img[i[0]-40:i[0]+40, i[1]-40:i[1]+40])
        return rects

    def processImage(self):
        img_num = 0
        for img in self.imgs[79:]:
            print self.num_imgs - img_num
            rect_num = 0
            rects = self.getCircles(img)
            length = len(rects)
            for rect in rects:
                failed = False
                f_name = str(str(img_num)+'_'+str(rect_num)+'.png')
                print length -rect_num
                while(1):
                    try:
                        cv2.imshow('is it a fish?', rect)
                        k = cv2.waitKey(33)
                    except:
                        failed = True
                        break
                    if k==1048603:    # Esc key to stop
                        break
                    elif k==-1:  # normally -1 returned,so don't print it
                        continue
                    elif k==1113938:
                        # Up pressed
                        f_name = OPEN+f_name
                        break
                    elif k==1113940:
                        # Down Pressed
                        f_name = CLOSE+f_name
                        break
                    elif k==1113937:
                        # Left pressed
                        f_name = DISCARD+f_name
                        break
                    elif k==1113939:
                        # Right pressed
                        f_name = NEGATIVE+f_name
                        break
                if not failed:
                    cv2.imwrite(f_name, rect)       
                rect_num+=1
                cv2.destroyAllWindows()
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
    import IPython; IPython.embed()
    


