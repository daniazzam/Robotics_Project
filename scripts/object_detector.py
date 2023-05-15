#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import actionlib
import robotics_project.msg

class Detect_parts:
	def __init__(self, name):
		rospy.init_node("Vision_Node")
		self.name = name
		self.bridge = CvBridge()
		self.image_pub = rospy.Publisher("image_output", Image, queue_size=1)
		self.image_sub = rospy.Subscriber("conveyor/camera1/image_raw", Image, self.callback)
		self.client = actionlib.SimpleActionClient(self.name, robotics_project.msg.GrabObjectAction)
		self.client.wait_for_server()
		self.detected = False
		self.detected_object = ""
		self.detected_position = 0
		self.prev_detected_position = 0
		self.prev_detected_object = ""
		self.picking_started = False
		self.frame_width = 800
		self.frame_height = 250
		self.size = (self.frame_width, self.frame_height)
		self.result = cv2.VideoWriter('filename.avi', cv2.VideoWriter_fourcc(*'MJPG'), 10, self.size)
		
	def callback(self, data):
		try:
			cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
			cv_image = cv_image[300:550, 0:800]
		except CvBridgeError as e:
			print(e)

		self.detected = False

		gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

		blurred = cv2.GaussianBlur(gray_image, (5, 5), 0)

		edges = cv2.Canny(blurred, 50, 150)

		_, contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		for pic, contour in enumerate(contours):
			area = cv2.contourArea(contour)
			if(area > 1200):
				x, y, w, h = cv2.boundingRect(contour)
				self.detected_position = x + w/2
				self.detected = True
				#cv2.putText(cv_image, str(area), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
				if 2400<area<2600:
					cv2.putText(cv_image, "Gear", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
					self.detected_object = "Gear"
					cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
					cv_image = cv2.circle(cv_image, (x + w/2, y + h/2), 4, (0, 255, 0), -1)
				elif 1700<area<1900:
					cv2.putText(cv_image, "Piston", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
					self.detected_object = "Piston"
					cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
					cv_image = cv2.circle(cv_image, (x + w/2, y + h/2), 4, (0, 255, 0), -1)
				elif 3200<area<3400:
					cv2.putText(cv_image, "Gasket", (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
					self.detected_object = "Gasket"
					cv_image = cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 0, 255), 2)
					cv_image = cv2.circle(cv_image, (x + w/2, y + h/2), 4, (0, 255, 0), -1)

		if self.detected:
			if self.prev_detected_object != self.detected_object:
				self.prev_detected_object = self.detected_object
				self.prev_detected_position = self.detected_position
				self.picking_started = False
				goal = robotics_project.msg.GrabObjectGoal()
				goal.object = self.detected_object
				goal.operation = "pre_position"
				self.client.send_goal(goal)
				print("Service Called pre-position")
			else:
				if self.detected_position > self.prev_detected_position:
					if self.detected_position >= 705 and (not self.picking_started):
						print("Arm is picking the object of the conveyor")
						self.prev_detected_position = self.detected_position
						self.picking_started = True
						goal = robotics_project.msg.GrabObjectGoal()
						goal.object = self.detected_object
						goal.operation = "PickUp"
						self.client.send_goal(goal)
				else:
					self.prev_detected_position = self.detected_position
					self.picking_started = False
					goal = robotics_project.msg.GrabObjectGoal()
					goal.object = self.detected_object
					goal.operation = "pre_position"
					self.client.send_goal(goal)
					print("Another object of the same type was detected")
					print("Calling pre-position")
		self.result.write(cv_image)
		try:
			self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
		except CvBridgeError as e:
			print(e)

if __name__ == '__main__':
	try:
		rospy.init_node("Vision_Node")
		Detect_parts("ariac/arm2/arm2_manipulation")
		rospy.spin()
	except rospy.ROSInterruptException as z:
		pass





