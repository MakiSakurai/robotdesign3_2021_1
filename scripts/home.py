#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sys
import moveit_commander
import actionlib
import geometry_msgs.msg
import rosnode
from tf import transformations
from tf.transformations import quaternion_from_euler
import math
from std_msgs.msg import Float64
from control_msgs.msg import (
    GripperCommandAction,
    GripperCommandGoal
)
class Home(object):

    def __init__(self):
        rospy.init_node("pose_groupstate_example")
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self._goal = GripperCommandGoal()
        self.robot = moveit_commander.RobotCommander()
        # Wait 10 Seconds for the gripper action server to start or exit
        self._client.wait_for_server(rospy.Duration(10.0))
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("Exiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()

    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)

    def feedback(self,msg):
        print("feedback callback")
        print(msg)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=0.1 ):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))
        return self._client.get_result()

    def clear(self):
        self._goal = GripperCommandGoal()

def main():
    gc = Home()
    arm = moveit_commander.MoveGroupCommander("arm")
    arm.set_max_velocity_scaling_factor(0.1)
    arm.set_max_acceleration_scaling_factor(1.0)
    gripper = moveit_commander.MoveGroupCommander("gripper")
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(1.0)
    target_joint_values = arm.get_current_joint_values()
    while len([s for s in rosnode.get_node_names() if 'rviz' in s]) == 0:
        rospy.sleep(1.0)
    rospy.sleep(1.0)
    key = raw_input()

    if key == "o": 
        print("Open Gripper.")
        gripper = 45.0
        gc.command(math.radians(gripper),1.0)
        result = gc.wait(2.0)
        print(result)
        rospy.sleep(1.0)
        print("")

    if key == "c": 
        print("Close Gripper.")
        gripper = 0.0 # 0.0
        gc.command(math.radians(gripper),1.0)
        result = gc.wait(2.0)
        print(result)
        rospy.sleep(1.0)
    

    if key == "h":    
        arm.set_max_velocity_scaling_factor(0.25)
        arm.set_max_acceleration_scaling_factor(0.5)
        print("vertical")
        arm.set_named_target("vertical")
        arm.go()
        target_joint_values = arm.get_current_joint_values()
        target_joint_values[1] = math.radians(-10)
        target_joint_values[2] = math.radians(0)
        target_joint_values[3] = math.radians(-80)
        target_joint_values[4] = math.radians(0)
        target_joint_values[5] = math.radians(-90)
        target_joint_values[6] = math.radians(90)
        arm.set_joint_value_target(target_joint_values)
        arm.go()
        print("current_joint_values (radians):")

        arm_goal_pose = arm.get_current_pose().pose
        print("Arm goal pose:")
        print(arm_goal_pose)

    if key == "f":
        print("vertical")
        arm.set_named_target("vertical")
        arm.go()

if __name__ == '__main__':
    while not rospy.is_shutdown():
            main()
