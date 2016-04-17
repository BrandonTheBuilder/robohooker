import os, sys
import cv2
dirname = os.path.dirname(__file__)
close = 'close'
c_file = open(close+'.dat', 'w')
for root, dirs, files in os.walk(os.path.join(dirname, close)):
    for name in files:
        try:
            img = cv2.imread(os.path.join(root, name))
            rect = [0,0,img.shape[1], img.shape[0]]
            c_file.write(os.path.join(root, name) +' 1 {} {} {} {} \n'.format(*rect))
        except Exception as ex:
            print ex
c_file.close()
op = 'open'
o_file= open(op+'.dat', 'w')
for root, dirs, files in os.walk(os.path.join(dirname, op)):
    for name in files:
        try:
            img = cv2.imread(os.path.join(root, name))
            rect = [0,0,img.shape[1], img.shape[0]]
            o_file.write(os.path.join(root, name) +' 1 {} {} {} {} \n'.format(*rect))
        except Exception as ex:
            print ex
o_file.close()

neg = 'negative'
o_file= open(neg+'.txt', 'w')
for root, dirs, files in os.walk(os.path.join(dirname, neg)):
    for name in files:
        try:
            img = cv2.imread(os.path.join(root, name))
            rect = [0,0,img.shape[1], img.shape[0]]
            o_file.write(os.path.join(root, name)+'\n')
        except Exception as ex:
            print ex
o_file.close()