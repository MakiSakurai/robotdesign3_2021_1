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
from geometry_msgs.msg import Point
class Home(object):
    def __init__(self):
        rospy.init_node("Pose_MediaPipe")
        rospy.Subscriber('/hand_topic', Point, self.callback, queue_size=1)
        self._client = actionlib.SimpleActionClient("/crane_x7/gripper_controller/gripper_cmd",GripperCommandAction)
        self._goal = GripperCommandGoal()
        self.robot = moveit_commander.RobotCommander()
        # Wait 10 Seconds for the gripper action server to start or exit
        self._client.wait_for_server(rospy.Duration(10.0))
        if not self._client.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("txiting - Gripper Action Server Not Found.")
            rospy.signal_shutdown("Action Server not found.")
            sys.exit(1)
        self.clear()
        self.t_x = 0
        self.t_y = 0
        self.t_z = 0
    def command(self, position, effort):
        self._goal.command.position = position
        self._goal.command.max_effort = effort
        self._client.send_goal(self._goal,feedback_cb=self.feedback)
    def callback(self, data):
        self.t_x = data.x
        self.t_y = data.y
        self.t_z = data.z
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
    def manipulation(self, x, y):
        gc = Home()
        arm = moveit_commander.MoveGroupCommander("arm")
        #安全性のため速度を1/2ほどに設定
        arm.set_max_velocity_scaling_factor(0.5)
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
            target_joint_values = arm.get_current_joint_values()
            target_joint_values[1] = math.radians(0)
            target_joint_values[2] = math.radians(0)
            target_joint_values[3] = math.radians(-90)
            target_joint_values[4] = math.radians(0)
            target_joint_values[5] = math.radians(-80)
            target_joint_values[6] = math.radians(90)
            arm.set_joint_value_target(target_joint_values)
            arm.go()
            print("current_joint_values (radians):")
            arm_goal_pose = arm.get_current_pose().pose
            #print("Arm goal pose:")
            #print(arm_goal_pose)
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = 0.0
            target_pose.position.z = 0.3
            q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            target_joint_values[6] = math.radians(90)
            arm.set_pose_target( target_pose )
            arm.set_joint_value_target(target_joint_values)
            print("current_joint_values (radians):")
            arm_goal_pose = arm.get_current_pose().pose
            arm.go()
            print("Arm goal pose:")
            print(arm_goal_pose)
            #一度verticalの姿勢にする
            print("vertical")
            arm.set_named_target("vertical")
            arm.go()

        if key == "t":    #確認用
            target_pose = geometry_msgs.msg.Pose()
            target_joint_values = arm.get_current_joint_values()
            target_pose.position.x = 0.26 + self.t_y
            target_pose.position.y = self.t_x 
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.3
            q = quaternion_from_euler( -3.14, 0.0, -3.14/2.0 )
            #ここから下の４行が手首の角度に関わる。現在入っている値はホームポジの時の手先座標
            #q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
        
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            """
            target_pose.orientation.x = 0.653390349328
            target_pose.orientation.y = 0.705918276099
            target_pose.orientation.z = 0.0399071045366
            target_pose.orientation.w = 0.0133428352264
          """
            arm.set_pose_target( target_pose )
            rospy.sleep(1.0)
   
            arm.go()


        if key == "w":    #確認用
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = 0.3
            target_pose.position.y = -0.1
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.3
            q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            target_joint_values[6] = math.radians(90)
            arm.set_pose_target( target_pose )
            arm.set_joint_value_target(target_joint_values)
            arm.go()

        if key == "q":    #確認用
            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = -self.t_x + 0.3
            target_pose.position.y = -self.t_y
            print(target_pose.position.x, target_pose.position.y)
            target_pose.position.z = 0.3
            q = quaternion_from_euler(-3.14, 0.0, -3.14/2.0)  # 上方から掴みに行く場合
            target_pose.orientation.x = q[0]
            target_pose.orientation.y = q[1]
            target_pose.orientation.z = q[2]
            target_pose.orientation.w = q[3]
            target_joint_values[6] = math.radians(90)
            arm.set_pose_target( target_pose )
            arm.set_joint_value_target(target_joint_values)
            arm.go()


        if key == "f":
            print("vertical")
            arm.set_named_target("vertical")
            arm.go()
            
    def run(self):
        try:
            while not rospy.is_shutdown():
                self.manipulation(self.t_x, self.t_y)
        except rospy.ROSInterruptException:
            pass
if __name__ == '__main__':
    while not rospy.is_shutdown():
            Home().run()