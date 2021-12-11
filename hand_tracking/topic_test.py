#!/usr/bin/env python
import rospy, math
import sys
from geometry_msgs.msg import Point

def main():
    rospy.init_node("unko")
    point_pub = rospy.Publisher("anego", Point,queue_size=10)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        point = Point()
        point.x = 111
        point.y = 222
        point.z = 333
        point_pub.publish(point)
        r.sleep()

if __name__ == '__main__':
    main()
