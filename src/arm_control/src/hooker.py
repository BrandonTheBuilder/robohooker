#!/usr/bin/env python
import rospy
from arm_control.srv import catch
from arm_control.msg import arm_status
import time

WAIT = 0
TRAVEL = 2
GRAB = -2
CATCH_WAIT = 0.1
FISH_PILE = [-8, 18, 2]
SET_FISH = [-8, 18, -4]

class Hooker(object):
    def __init__(self):
        self.home()
        if __name__ == '__main__':
            self._setup_ros()


    def _setup_ros(self):
        self.service = rospy.Service("catch_fish", catch, self.catch)
        self.pub = rospy.Publisher("/arm_status", arm_status, queue_size=10)


    def publish_updates(self):
        rate = rospy.Rate(10)
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
        self.drop_time = msg.time
        self.position(msg.x, msg.y)
        return True


    def home(self):
        self.move_arm(0, 0, 0)
        self.status = 'home'
        self.ready = True


    def position(self, x, y):
        self.move_arm(x, y, WAIT)
        self.status = "Waiting"
        self.grab()


    def grab(self):
        while self.drop_time-rospy.Time.now().to_time() >= 0:
            pass
            # rospy.loginfo("On the corner")
        self.move_arm(self.current[0], self.current[1], GRAB)
        time.sleep(CATCH_WAIT)
        self.move_arm(self.current[0], self.current[1], TRAVEL)
        self.status = "Dropping"
        self.drop()


    def drop(self):
        self.move_arm(*FISH_PILE)
        self.move_arm(*SET_FISH)
        self.home()

        

    def move_arm(self, x, y, z):
        time.sleep(0.5)
        self.current = [x,y,z]
        rospy.logwarn('Moved to {}'.format(self.current))


if __name__ == '__main__':
    rospy.init_node('arm_control')
    h = Hooker()

    h.publish_updates()