#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, Quaternion
from osrf_gear.srv import VacuumGripperControl
import actionlib
import robotics_project.msg
from robotics_project.srv import SelectBox
from tf.transformations import quaternion_from_euler
import numpy as np

class Motion_planner3:
	def __init__(self, name):
		rospy.init_node("move_group_python_node")
		moveit_commander.roscpp_initialize(sys.argv)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("manipulator")
		self.trajectory_pub = rospy.Publisher("ariac/arm2/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)
		rospy.wait_for_service('gripper/control')
		self.gripper_prox = rospy.ServiceProxy('gripper/control', VacuumGripperControl)
		rospy.Service("select_box", SelectBox, self.select_box_callback)
		self.current_box = ""
		self.boxes = []
		self.object_to_pick = ""
		self._action_name = name
		self._as = actionlib.SimpleActionServer(self._action_name, robotics_project.msg.GrabObjectAction, execute_cb=self.execute_cb, auto_start = False)
		self._as.start()

		self.group.clear_pose_targets()
		joint_values = self.group.get_current_joint_values()
		joint_values[1] = 0.0
		joint_values[0] = 0.0
		self.group.set_joint_value_target(joint_values)
		self.group.plan()
		self.group.go(wait=True)
		rospy.sleep(0.75)

		self.group.clear_pose_targets()
		target_pose = Pose()
		target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), np.deg2rad(-30.0)))
		target_pose.position.x = 1.1
		target_pose.position.y = -1.0
		target_pose.position.z = 1.225

		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=True)
		rospy.sleep(0.75)

		target_pose = Pose()
		target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
		target_pose.position.x = 1.1
		target_pose.position.y = -1.0
		target_pose.position.z = 0.95 #0.935 0.92

		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.gripper_prox(True)
		self.group.go(wait=True)
		rospy.sleep(0.75)

		target_pose.position.x = 1.1
		target_pose.position.y = -1.0
		target_pose.position.z = 1.225
		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=True)
		rospy.sleep(0.75)


	def execute_cb(self, goal):
		Delay = 0.1
		if goal.operation == "pre_position":
			self.object_to_pick = goal.object
			self.group.clear_pose_targets()
			joint_values = self.group.get_current_joint_values()
			joint_values[1] = 0.0
			joint_values[0] = 0.0
			self.group.set_joint_value_target(joint_values)
			self.group.plan()
			self.group.go(wait=True)
			#rospy.sleep(Delay)

			self.group.clear_pose_targets()
			target_pose = Pose()
			target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), np.deg2rad(-30.0)))
			target_pose.position.x = 1.1
			target_pose.position.y = -1.0
			target_pose.position.z = 1.225

			self.group.set_pose_target(target_pose)
			self.group.plan()
			self.group.go(wait=True)
			#rospy.sleep(Delay)
			
		elif goal.operation == "PickUp":
			self.object_to_pick = goal.object
			if self.object_to_pick == "Gear":
				target_pose = Pose()
				target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
				target_pose.position.x = 1.1
				target_pose.position.y = -1.0
				target_pose.position.z = 0.9305 #0.935 0.92

				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.gripper_prox(True)
				self.group.go(wait=True)
				rospy.sleep(Delay)

				target_pose.position.x = 1.1
				target_pose.position.y = -1.0
				target_pose.position.z = 1.225
				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.group.go(wait=True)
				rospy.sleep(Delay)

			elif self.object_to_pick == "Gasket":
				target_pose = Pose()
				target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
				target_pose.position.x = 1.1
				target_pose.position.y = -1.0
				target_pose.position.z = 0.9375 #0.935 0.92

				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.gripper_prox(True)
				self.group.go(wait=True)
				rospy.sleep(Delay)

				target_pose.position.x = 1.1
				target_pose.position.y = -1.0
				target_pose.position.z = 1.225
				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.group.go(wait=True)
				rospy.sleep(Delay)

			elif self.object_to_pick == "Piston":
				target_pose = Pose()
				target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
				target_pose.position.x = 1.1
				target_pose.position.y = -1.0
				target_pose.position.z = 0.925 #0.935 0.92

				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.gripper_prox(True)
				self.group.go(wait=True)
				rospy.sleep(Delay)

				target_pose.position.x = 1.15
				target_pose.position.y = -1.0
				target_pose.position.z = 1.225
				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.group.go(wait=True)
				rospy.sleep(Delay)

			self.current_box = self.boxes.pop(0)
			if self.current_box == "Box1":
				self.group.clear_pose_targets()
				joint_values = self.group.get_current_joint_values()
				joint_values[1] = -3.14
				#joint_values[0] = -0.3######
				self.group.set_joint_value_target(joint_values)
				self.group.plan()
				self.group.go(wait=True)
				rospy.sleep(Delay)

				target_pose = Pose()
				target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), np.deg2rad(30.0)))
				target_pose.position.x = -0.3
				target_pose.position.y = -0.383
				target_pose.position.z = 1.2


				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.group.go(wait=True)
				self.gripper_prox(False)
				rospy.sleep(Delay)

			elif self.current_box == "Box2":
				self.group.clear_pose_targets()
				joint_values = self.group.get_current_joint_values()
				joint_values[1] = -3.14
				#joint_values[0] = -1.1######
				self.group.set_joint_value_target(joint_values)
				self.group.plan()
				self.group.go(wait=True)
				rospy.sleep(Delay)

				target_pose = Pose()
				target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
				target_pose.position.x = -0.3
				target_pose.position.y = -1.15
				target_pose.position.z = 1.2


				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.group.go(wait=True)
				self.gripper_prox(False)
				rospy.sleep(Delay)

			elif self.current_box == "Box3":
				self.group.clear_pose_targets()
				joint_values = self.group.get_current_joint_values()
				joint_values[1] = -2.6
				#joint_values[0] = -2.3
				self.group.set_joint_value_target(joint_values)
				self.group.plan()
				self.group.go(wait=True)
				rospy.sleep(Delay)

				target_pose = Pose()
				target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
				target_pose.position.x = -0.3
				target_pose.position.y = -1.916
				target_pose.position.z = 1.2

				self.group.set_pose_target(target_pose)
				self.group.plan()
				self.group.go(wait=True)
				self.gripper_prox(False)
				rospy.sleep(Delay)	

			self.group.clear_pose_targets()
			joint_values = self.group.get_current_joint_values()
			joint_values[1] = 0.0
			joint_values[0] = 0.0
			self.group.set_joint_value_target(joint_values)
			self.group.plan()
			self.group.go(wait=True)
			rospy.sleep(Delay)

			self.group.clear_pose_targets()
			target_pose = Pose()
			target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), np.deg2rad(-30.0)))
			target_pose.position.x = 1.1
			target_pose.position.y = -1.0
			target_pose.position.z = 1.225

			self.group.set_pose_target(target_pose)
			self.group.plan()
			self.group.go(wait=True)
			rospy.sleep(Delay)		

		result = robotics_project.msg.GrabObjectResult()
		result.success = True
		self._as.set_succeeded(result)


	def select_box_callback(self, req):
		self.boxes.append(req.box)
		rospy.loginfo(req.box)
		return req.box


if __name__ == '__main__':
	Motion_planner3("arm2_manipulation")
	rospy.spin()






