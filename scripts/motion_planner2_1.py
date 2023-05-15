#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, Quaternion
from osrf_gear.srv import VacuumGripperControl
from tf.transformations import quaternion_from_euler
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_node")
rospy.wait_for_service('gripper/control')
gripper_prox = rospy.ServiceProxy('gripper/control', VacuumGripperControl)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

trajectory_pub = rospy.Publisher("ariac/arm2/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)

group.clear_pose_targets()
joint_values = group.get_current_joint_values()
joint_values[1] = 0.0
joint_values[0] = 0.0
group.set_joint_value_target(joint_values)
plan2 = group.plan()
group.go(wait=True)
rospy.sleep(0.75)

group.clear_pose_targets()
target_pose = Pose()
target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
target_pose.position.x = 1.1
target_pose.position.y = -1.0
target_pose.position.z = 1.225

group.set_pose_target(target_pose)
group.plan()
group.go(wait=True)
rospy.sleep(0.75)

moveit_commander.roscpp_shutdown()





