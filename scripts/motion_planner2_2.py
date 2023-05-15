#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, Quaternion
from osrf_gear.srv import VacuumGripperControl
from robotics_project.srv import PickObject
from tf.transformations import quaternion_from_euler
import numpy as np

class Motion_planner2:
	def __init__(self):		
		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node("move_group_python_node")
		rospy.wait_for_service('gripper/control')
		self.gripper_prox = rospy.ServiceProxy('gripper/control', VacuumGripperControl)
		self.object_to_pick = ""

		rospy.Service("pre_position", PickObject, self.pre_callback)

		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("manipulator")

		self.trajectory_pub = rospy.Publisher("ariac/arm2/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)

	def pre_callback(self, req):
		self.object_to_pick = req.object
		self.group.clear_pose_targets()
		joint_values = self.group.get_current_joint_values()
		joint_values[1] = 0.0
		joint_values[0] = 0.0
		self.group.set_joint_value_target(joint_values)
		self.group.plan()
		self.group.go(wait=False)
		#rospy.sleep(0.75)

		#self.group.clear_pose_targets()
		target_pose = Pose()
		target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
		target_pose.position.x = 1.1
		target_pose.position.y = -1.0
		target_pose.position.z = 1.225

		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=False)
		#rospy.sleep(0.75)
		return True


if __name__ == '__main__':
	Motion_planner2()
	rospy.spin()




