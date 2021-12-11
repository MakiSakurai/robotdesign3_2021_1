#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import math
import tf2_ros
from geometry_msgs.msg import Point

class coordinate():
    def __init__(self):
        rospy.init_node('MediaPipe_TF')
        rospy.Subscriber('/hand_topic', Point, self.callback, queue_size=1)
        self.t_x = 0
        self.t_y = 0
        self.t_z = 0
        
    def callback(self, data):
        self.t_x = data.x
        self.t_y = data.y
        self.t_z = data.z

    def loop(self):
        rate = rospy.Rate(10.0)
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)    
        while not rospy.is_shutdown():
            try:
                trans = tfBuffer.lookup_transform("base_link", "camera_base", rospy.Time(0))
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                print("I'm not getting TF.")      
                continue

            rate.sleep()    

            plus_x = trans.transform.translation.x + self.t_x
            plus_y = trans.transform.translation.y + self.t_y
            plus_z = trans.transform.translation.z + self.t_z
            print(plus_x, plus_y, plus_z)




if __name__ == '__main__': 
    makimaki = coordinate()  
    makimaki.loop()