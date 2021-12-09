#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import math
import tf2_ros
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

if __name__ == '__main__':
    rospy.init_node('hoge')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rate = rospy.Rate(10.0)
    rate.sleep()
    #listener = tf.TransformListener() #TransformListenerのオブジェクトを作る。一旦listenerが作成されると、ケーブルを通してtfのtransformationsを受け取り始め、１０秒間それらをバッファにためる。
    #listener.waitForTransform("base_link", "camera_base", rospy.Time(), rospy.Duration(4.0)) #リスナーの作成後すぐにトランスフォームを要求しないようにwait
    
    while not rospy.is_shutdown():
        Media_hand = rospy.Publisher('anego', geometry_msgs.msg.Point, queue_size=1)
        trans = tfBuffer.lookup_transform("base_link", "camera_base", rospy.Time(0))
        rate.sleep()
        msg = geometry_msgs.msg.Point()
        Media_hand.publish(msg)
        print(trans.transform.translation.x)

        rate.sleep()