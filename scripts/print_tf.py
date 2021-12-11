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
        self.t_x = data.x
        self.t_y = data.y
        self.t_z = data.z

    def loop(self):
        rate = rospy.Rate(10.0)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)
        try:
            trans = tfBuffer.lookup_transform("base_link", "camera_base", rospy.Time(0))
        except Exception as e:
            print("muri")
        rate.sleep()
        plus_x = trans.transform.translation.x + self.t_x
        plus_y = trans.transform.translation.y + self.t_y
        plus_z = trans.transform.translation.z + self.t_z
        print(plus_x, plus_y, plus_z)


def main():
    makimaki = tekitou()
    

    while not rospy.is_shutdown():
        makimaki.loop()
 
            
if __name__ == '__main__':   
    main()