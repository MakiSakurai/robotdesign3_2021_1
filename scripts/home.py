#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
import geometry_msgs.msg
import rosnode
from tf import transformations
from tf.transformations import quaternion_from_euler
import math


def main():
    rospy.init_node("pose_groupstate_example")
    robot = moveit_commander.RobotCommander()
    arm = moveit_commander.MoveGroupCommander("arm")
    #安全性のため速度を1/2ほどに設定
    arm.set_max_velocity_scaling_factor(0.25)
    arm.set_max_acceleration_scaling_factor(0.5)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(1.0)
    target_joint_values = arm.get_current_joint_values()

    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)

    key = raw_input()
    print(key)

    if key == "h":    
        #一度verticalの姿勢にする
        print("vertical")
        arm.set_named_target("vertical")
        arm.go()

        """""
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.
        target_pose.position.y = 0.1
        target_pose.position.z = 0.6
        q = quaternion_from_euler( 0.0, 0.0, 0.0 )
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target( target_pose )
        arm.go()
        #target_joint_values[5] = math.radians(-30)
        #arm.set_joint_value_target(target_joint_values)
        #arm.go()
        print("gre")
        """
        target_joint_values = arm.get_current_joint_values()
        target_joint_values[1] = math.radians(-10)
        target_joint_values[2] = math.radians(0)
        target_joint_values[3] = math.radians(-80)
        target_joint_values[4] = math.radians(0)
        target_joint_values[5] = math.radians(-90)
        target_joint_values[6] = math.radians(0)
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print("current_joint_values (radians):")

        arm_goal_pose = arm.get_current_pose().pose
        print("Arm goal pose:")
        print(arm_goal_pose)
    
    if key == "t":    #確認用
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = 0.1
        target_pose.position.z = 0.3
        q = quaternion_from_euler( 0.0, 0.0, 0.0 )
        #ここから下の４行が手首の角度に関わる。現在入っている値はホームポジの時の手先座標
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = -0.999909353162
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0133428352264
        arm.set_pose_target( target_pose )
        arm.go()
        
    if key == "y":    #確認用
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.2
        target_pose.position.y = 0.2
        target_pose.position.z = 0.3
        q = quaternion_from_euler( 0.0, 0.0, 0.0 )
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = -0.999909353162
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0133428352264
        arm.set_pose_target( target_pose )
        arm.go()

    if key == "x":    #確認用
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = 0.2
        target_pose.position.z = 0.3
        q = quaternion_from_euler( 0.0, 0.0, 0.0 )
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = -0.999909353162
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0133428352264
        arm.set_pose_target( target_pose )
        arm.go()    

    if key == "Y":    #確認用
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.3
        target_pose.position.y = -0.2
        target_pose.position.z = 0.3
        q = quaternion_from_euler( 0.0, 0.0, 0.0 )
        target_pose.orientation.x = 0.0
        target_pose.orientation.y = -0.999909353162
        target_pose.orientation.z = 0.0
        target_pose.orientation.w = 0.0133428352264
        arm.set_pose_target( target_pose )
        arm.go()   

    """""
    if key == "z":    
        target_pose = geometry_msgs.msg.Pose()
        target_pose.position.x = 0.0
        target_pose.position.y = 2.0
        target_pose.position.z = 0.0
        q = quaternion_from_euler( 0.0, 0.0, 0.0 )
        target_pose.orientation.x = q[0]
        target_pose.orientation.y = q[1]
        target_pose.orientation.z = q[2]
        target_pose.orientation.w = q[3]
        arm.set_pose_target( target_pose )
        arm.go()
    arm_goal_pose = arm.get_current_pose().pose
    print("Arm goal pose:")
    print(arm_goal_pose)
    print("done")
    """
    if key == "f":
        print("vertical")
        arm.set_named_target("vertical")
        arm.go()

if __name__ == '__main__':
    while not rospy.is_shutdown():
            main()
