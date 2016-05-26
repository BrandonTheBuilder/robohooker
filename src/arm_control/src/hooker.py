#!/usr/bin/env python
import rospy
from arm_control.srv import catch
from arm_control.msg import arm_status
import time
import math
import serial
import sys

PORT='/dev/ttyACM0'
WAIT = -4
TRAVEL = 0
GRAB = -8
CATCH_WAIT = 0.5
FISH_PILE = [-15, -15, -6]
SET_FISH = [-15, -15, -10]

A=9.1
F=16.5
L=13.3
PI=3.14159
Tr_off=45
T1_off=0
T2_off=45
#end constant defs
delta=5

class Pos:
    x=0.0
    y=0.0
    z=0.0

class Hooker(object):
    def __init__(self):
        self.build_table()
        self.resetCount = 0
        self.ser= serial.Serial(PORT, 115200,timeout=1)
        self.home()
        if __name__ == '__main__':
            self._setup_ros()


    def _setup_ros(self):
        self.service = rospy.Service("catch_fish", catch, self.catch)
        self.pub = rospy.Publisher("/arm_status", arm_status, queue_size=1)


    def build_table(self):
        maxX=-100
        maxY=-100
        maxZ=-100

        minX=100
        minY=100
        minZ=100
        self.Table = [[[Pos() for j in range(91)] for i in range(121)] for k in range(91)]
        rospy.loginfo('Building Table')
        for i in range(0,91):
            rospy.loginfo('---')
            for j in range(0,121):
                for k in range(0,91):
                    iR=(float(i-Tr_off)/360)*2*PI
                    jR=(float(j-T1_off)/360)*2*PI
                    kR=(float(k-T2_off)/360)*2*PI

                    D=A+A*math.cos(jR)+F*math.cos(kR)
                    self.Table[i][j][k].y=math.cos(iR)*D-25.59
                    self.Table[i][j][k].x=math.sin(iR)*D   #-15.35
                    self.Table[i][j][k].z=A*math.sin(jR)+F*math.sin(kR)-L+3.98

                    if self.Table[i][j][k].y>maxY:
                        maxY=self.Table[i][j][k].y
                    if self.Table[i][j][k].y<minY:
                        minY=self.Table[i][j][k].y
                        
                    if self.Table[i][j][k].x>maxX:
                        maxX=self.Table[i][j][k].x
                    if self.Table[i][j][k].x<minX:
                        minX=self.Table[i][j][k].x
                        
                    if self.Table[i][j][k].z>maxZ:
                        maxZ=self.Table[i][j][k].z
                    if self.Table[i][j][k].z<minZ:
                        minZ=self.Table[i][j][k].z
                    
                       
                        #print ("X: " + str(Table[i][j][k].x) +" Y: " + str(Table[i][j][k].y) +" Z: " + str(Table[i][j][k].z))
        self.max_x = maxX
        self.min_x = minX
        self.max_y = maxY
        self.min_y = minY
        self.max_z = maxZ
        self.min_z = minZ
        rospy.loginfo("Table Complete")


    def lookup(self, nx,ny,nz):
        deltaLow=10;
        # print ("Looking for X: " + str(nx) +" Y: " + str(ny) +" Z: " + str(nz))
        for i in range(0,91):
            for j in range(0,120):
                for k in range(0,91):
                    
                    if abs(nx-self.Table[i][j][k].x)<delta and abs(ny-self.Table[i][j][k].y)<delta and abs(nz-self.Table[i][j][k].z)<delta:
                        
                        deltaTemp=abs(nx-self.Table[i][j][k].x)+abs(ny-self.Table[i][j][k].y)+abs(nz-self.Table[i][j][k].z)
                        deltaTemp=deltaTemp/3
                        if deltaTemp<deltaLow:
                            deltaLow=deltaTemp
                            Pi=i
                            Pj=j
                            Pk=k
                        
        if deltaLow<2:
            # print("Delta of: "+str(deltaLow))
            out=str(Pi+Tr_off)+","+str(Pj-T1_off)+","+str(Pk-T2_off)
            # print("Found it: Tr-> "+ str(Pi-Tr_off) + " T1-> " + str(Pj-T1_off) + " T2-> " + str(Pk-T2_off))
            rospy.loginfo(out)
            self.ser.write(bytearray(out,'utf-8'))
            time.sleep(1)
            rece='NO'
            received= str(self.ser.readline())
            # if(sys.getsizeof(received)>52):
            #     rece=received[2]+received[3]
            # if(rece=='OK'):
            #     print("Got it!!!  Reset Count: "+str(self.resetCount))
            # else:
            #     self.resetCount=self.resetCount+1
            #     print("Reseting....."+str(self.resetCount))
            #     self.ser.close()
            #     self.ser.open()
            return
        print("Point not found...")


    def publish_updates(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            status = arm_status()
            status.status = self.status
            status.current = self.current
            status.ready = self.ready
            # status.wait_time = self.drop_time-rospy.Time.now().to_time()
            self.pub.publish(status)
            rate.sleep()


    def catch(self, msg):
        self.ready = False
        self.status = "Positioning"
        self.drop_time = msg.time + rospy.get_time()
        # if self.drop_time - rospy.get_time() >= 5.0:
        #     rospy.logwarn('Wait time too long')
        #     self.ready = True
        #     return False
        if msg.x > self.max_x or msg.x < self.min_x:
            rospy.logwarn('x out of bounds')
            self.ready = True
            return False
        elif msg.y > self.max_y or msg.y < self.min_y:
            rospy.logwarn('y out of bounds')
            self.ready = True
            return False
        else:
            self.position(msg.x, msg.y)
            return True


    def home(self):
        self.move_arm(0.0, 0.0, 0.0)
        time.sleep(1)
        self.status = 'home'
        self.ready = True


    def position(self, x, y):
        self.move_arm(x, y, WAIT)
        self.status = "Waiting"
        self.grab()


    def grab(self):
        # if self.drop_time == 0.0:
        #     time.sleep(2)
        
        while self.drop_time-rospy.get_time() >= 0:
            rospy.loginfo('Wating {}'.format(self.drop_time-rospy.get_time()))
        self.move_arm(self.current[0], self.current[1], GRAB)
        time.sleep(CATCH_WAIT)
        self.move_arm(self.current[0], self.current[1], TRAVEL)
        self.status = "Dropping"
        self.drop()


    def drop(self):
        self.move_arm(*FISH_PILE)
        self.move_arm(*SET_FISH)
        self.move_arm(*FISH_PILE)
        self.move_arm(*SET_FISH)
        self.move_arm(*FISH_PILE)
        self.home()

        

    def move_arm(self, x, y, z):
        self.lookup(x,y,z)
        self.current = [x,y,z]
        rospy.logwarn('Moved to {}'.format(self.current))


if __name__ == '__main__':
    rospy.init_node('arm_control')
    h = Hooker()

    h.publish_updates()