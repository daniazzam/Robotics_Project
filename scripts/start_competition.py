#!/usr/bin/env python
import rospy
from osrf_gear.srv import ConveyorBeltControl

def callback(req):
	global prox
	return prox(req)


rospy.init_node("start_comp")
rospy.wait_for_service("ariac/conveyor/control")
prox = rospy.ServiceProxy("ariac/conveyor/control", ConveyorBeltControl)
rospy.Service("ariac/arm2/conveyor/control", ConveyorBeltControl, callback)
rospy.spin()
