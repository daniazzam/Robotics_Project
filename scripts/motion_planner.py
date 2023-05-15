#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, Quaternion
from tf.transformations import quaternion_from_euler
from osrf_gear.srv import ConveyorBeltControl
from osrf_gear.srv import VacuumGripperControl
import numpy as np


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_node")
rospy.wait_for_service("conveyor/control")
belt_prox = rospy.ServiceProxy("conveyor/control", ConveyorBeltControl)
rospy.wait_for_service('gripper/control')
gripper_prox = rospy.ServiceProxy('gripper/control', VacuumGripperControl)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

trajectory_pub = rospy.Publisher("ariac/arm2/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)

target_pose = Pose()
target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
target_pose.position.x = 1.1
target_pose.position.y = -1.0
target_pose.position.z = 0.935 #0.935 0.92

belt_prox(100)
group.set_pose_target(target_pose)
plan1 = group.plan()
gripper_prox(True)
group.go(wait=True)


rospy.sleep(0.75)

target_pose.position.x = 1.1
target_pose.position.y = -1.0
target_pose.position.z = 1.225
group.set_pose_target(target_pose)
plan2 = group.plan()
group.go(wait=True)
belt_prox(0)
gripper_prox(False)
rospy.sleep(0.75)

moveit_commander.roscpp_shutdown()




