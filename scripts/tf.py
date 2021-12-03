#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import moveit_commander
import geometry_msgs.msg
import rosnode
from geometry_msgs.msg import Point
from tf.transformations import quaternion_from_euler

def main():

    rospy.init_node("anego")
    pub = rospy.Subscriber("hand_topic", Point, queue_size=10)

    listener = tf.TransformListener()
    (trans,rot) = listener.lookupTransform('/base_link', '/camera_base', rospy.Time(0))
    kekka = listener.fromTranslationRotation(trans, rot)
    rospy.loginfo(kekka)

if __name__ == '__main__':
    try:
        if not rospy.is_shutdown():
            main()
    except rospy.ROSInterruptException:
        pass