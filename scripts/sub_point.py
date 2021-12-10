#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import math
import tf2_ros
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

class tekitou():
    def __init__(self):
        rospy.init_node('hoge')
        rospy.Subscriber('/hand_topic', Point, self.callback, queue_size=1)

    def callback(self, data):
        t_x = data.x
        t_y = data.y
        t_z = data.z
        print(t_x)

    def loop(self):
        print("makki")


def main():

    makimaki = tekitou()
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        makimaki.loop()
        rate.sleep()


            
if __name__ == '__main__':   
    main()