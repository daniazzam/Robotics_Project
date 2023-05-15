#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, Quaternion
from osrf_gear.srv import VacuumGripperControl
from tf.transformations import quaternion_from_euler
from robotics_project.srv import Respawn,RespawnRequest, RespawnResponse
from library.PriorityQueue import PriorityQueue
# from library.Order import Order
from robotics_project.srv import requestedorder,requestedorderRequest,requestedorderResponse
from robotics_project.srv import getorder,getorderRequest,getorderResponse
from robotics_project.srv import SelectBox

import random
import numpy as np

class Order_Class:
	def __init__(self,package_order,ID):
		self.Package_order=package_order
		self.id=ID

class Motion:
	def __init__(self):
      

		moveit_commander.roscpp_initialize(sys.argv)
		rospy.init_node("move_group_python_node_2")
		rospy.wait_for_service('gripper/control')
		rospy.loginfo('Pre service')
		rospy.wait_for_service('Respawn_objects')
		rospy.loginfo('khara service')
		rospy.wait_for_service('select_box')
		rospy.loginfo('Post Service')
		rospy.Service("Add_Order",requestedorder,self.add_callback)
		rospy.Service("Get_Order",getorder,self.execute_orders)
		self.gripper_prox = rospy.ServiceProxy('gripper/control', VacuumGripperControl)
		self.box_prox = rospy.ServiceProxy('select_box', SelectBox)
		self.respawn_request=rospy.ServiceProxy('Respawn_objects', Respawn)
		self.robot = moveit_commander.RobotCommander()
		self.scene = moveit_commander.PlanningSceneInterface()
		self.group = moveit_commander.MoveGroupCommander("manipulator")
		self.trajectory_pub = rospy.Publisher("ariac/arm1/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)
		
		self.gasket_1 = [-0.15, 2.07, 0.758]
		self.gasket_2 = [-0.15, 1.76, 0.758]
		self.gasket_3 = [-0.45, 2.07, 0.758]
		self.gasket_4 = [-0.45, 1.76, 0.758]
		self.Gaskets = [self.gasket_1, self.gasket_2, self.gasket_3, self.gasket_4]
		self.gear_1 = [-0.15, 1.3, 0.7415]
		self.gear_2 = [-0.15, 1, 0.7415]
		self.gear_3 = [-0.45, 1.3, 0.7415]
		self.gear_4 = [-0.45, 1, 0.7415]
		self.Gears = [self.gear_1, self.gear_2, self.gear_3, self.gear_4]
		self.piston_1 = [-0.15, 0.53, 0.74]
		self.piston_2 = [-0.15, 0.23, 0.74]
		self.piston_3 = [-0.45, 0.53, 0.74]
		self.piston_4 = [-0.45, 0.23, 0.74]
		self.Pistons = [self.piston_1, self.piston_2, self.piston_3, self.piston_4]
		
		self.prio_queue=PriorityQueue()
		self.Executing=False
		self.previous_order=None
		self.order_finished=False
		self.pointer_of_gaskets=0
		self.pointer_of_gears=0
		self.pointer_of_pistons=0
		#self.execute_orders()
		rospy.spin()

		
	def add_callback(self, request):
		gaskets = request.num_gaskets
		gears = request.num_gears
		pistons=request.num_pistons
		priority=request.priority
		i=0
		arr = []
		while i < gaskets:
			arr.append('Gasket')
			i+=1
		i=0
		while i < gears:
			arr.append('Gear')
			i+=1
		i=0
		while i < pistons:
			arr.append('Piston')
			i+=1
		order=Order_Class(arr,priority)
		self.prio_queue.push(order,priority)
		response=requestedorderResponse(True)
		return response
	
	def execute_orders(self,request):
		while(self.prio_queue.get_length()!=0):
			
			order=self.prio_queue.peek()
			arr = order.Package_order
			print(arr)

			if (len(arr) == 0):
				self.prio_queue.pop()#call service push box
				pass
			else:
				if order.id == 1:
					self.box_prox("Box1")
				elif order.id == 2:
					self.box_prox("Box2")
				elif order.id == 3:
					self.box_prox("Box3")

				element = arr.pop(0)
				if(element=="Gasket"):
						self.get_object(self.Gaskets[self.pointer_of_gaskets])
						self.pointer_of_gaskets+=1
						if(self.pointer_of_gaskets>=4):
							self.pointer_of_gaskets = 0
							response = self.respawn_request('Gasket')
				if(element=="Gear"):
						self.get_object(self.Gears[self.pointer_of_gears])
						self.pointer_of_gears+=1
						if(self.pointer_of_gears>=4):
							self.pointer_of_gears = 0
							response = self.respawn_request('Gear')
				if(element=="Piston"):
						self.get_object(self.Pistons[self.pointer_of_pistons])
						self.pointer_of_pistons+=1
						if(self.pointer_of_pistons>=4):
							self.pointer_of_pistons = 0
							response = self.respawn_request('Piston')

		response=getorderResponse(True)
		return response
                              

	def get_object(self,coordinates):
		
		# joint_values = group.get_current_joint_values()
		# joint_values[0] = joint
		# group.set_joint_value_target(joint_values)
		# group.plan()
		# group.go(wait=True)
		Delay = 0.75
		target_pose = Pose()    
		self.group.clear_pose_targets()
		joint_values = self.group.get_current_joint_values()
		joint_values[1] = 3.14
		self.group.set_joint_value_target(joint_values)
		self.group.plan()
		self.group.go(wait=True)
		rospy.sleep(Delay)

		target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), np.deg2rad(90.0)))
		target_pose.position.x = coordinates[0]
		target_pose.position.y = coordinates[1]
		target_pose.position.z = 1.2
		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=True)

		rospy.sleep(Delay)

		# Go to target position
		target_pose.position.x = coordinates[0]
		target_pose.position.y = coordinates[1]
		target_pose.position.z = coordinates[2]
		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=True)

		# Activate Gripper
		self.gripper_prox(True)
		rospy.sleep(Delay)

		target_pose.position.x = coordinates[0]
		target_pose.position.y = coordinates[1]
		target_pose.position.z = 1.2
		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=True)

		rospy.sleep(Delay)

		self.group.clear_pose_targets()
		joint_values = self.group.get_current_joint_values()
		joint_values[1] = 0.0
		self.group.set_joint_value_target(joint_values)
		self.group.plan()
		self.group.go(wait=True)

		rospy.sleep(Delay)

		# Place Object on Conveyor Belt
		target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
		target_pose.position.x = 1.1
		target_pose.position.y = 1.7
		target_pose.position.z = 0.98
		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=True)
		self.gripper_prox(False)
		rospy.sleep(Delay)

		# Shift gripper upwards
		target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
		target_pose.position.x = 1.1
		target_pose.position.y = 1.7
		target_pose.position.z = 1.2
		self.group.set_pose_target(target_pose)
		self.group.plan()
		self.group.go(wait=True)
		rospy.sleep(Delay)

		# Restore Default Position
		self.group.clear_pose_targets()
		joint_values = self.group.get_current_joint_values()
		joint_values[1] = 3.14
		self.group.set_joint_value_target(joint_values)
		self.group.plan()
		self.group.go(wait=True)
		rospy.sleep(Delay)

if __name__ == '__main__':
	try:
			Motion()
	except rospy.ROSInterruptException as e:
			pass
























