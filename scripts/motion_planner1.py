#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_node")

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

trajectory_pub = rospy.Publisher("ariac/arm1/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)
#group.clear_pose_targets()
joint_values = group.get_current_joint_values()
print(joint_values)
#joint_values[1] = 0.0
#group.set_joint_value_target(joint_values)
#plan2 = group.plan()
#print(group.go(wait=True))
#rospy.sleep(2)
moveit_commander.roscpp_shutdown()

