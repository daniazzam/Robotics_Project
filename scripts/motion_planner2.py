#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Pose, Quaternion
from osrf_gear.srv import VacuumGripperControl
from tf.transformations import quaternion_from_euler
import random
import numpy as np

moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_node")
rospy.wait_for_service('gripper/control')
gripper_prox = rospy.ServiceProxy('gripper/control', VacuumGripperControl)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

trajectory_pub = rospy.Publisher("ariac/arm1/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)

gasket_1 = [-0.15, 2.07, 0.758]
gasket_2 = [-0.15, 1.76, 0.758]
gasket_3 = [-0.45, 2.07, 0.758]
gasket_4 = [-0.45, 1.76, 0.758]
Gaskets = [gasket_1, gasket_2, gasket_3, gasket_4]

gear_1 = [-0.15, 1.3, 0.758]
gear_2 = [-0.15, 1, 0.758]
gear_3 = [-0.45, 1.3, 0.758]
gear_4 = [-0.45, 1, 0.758]
Gears = [gear_1, gear_2, gear_3, gear_4]

piston_1 = [-0.15, 0.53, 0.758]
piston_2 = [-0.15, 0.23, 0.758]
piston_3 = [-0.45, 0.53, 0.758]
piston_4 = [-0.45, 0.23, 0.758]
Pistons = [piston_1, piston_2, piston_3, piston_4]

group.clear_pose_targets()

joint_values = group.get_current_joint_values()
joint_values[0] = 0.5
group.set_joint_value_target(joint_values)

group.plan()
group.go(wait=True)
while len(Gaskets) > 0:
    rand = random.randint(0, len(Gaskets)-1)
    target_pose = Pose()
    target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), np.deg2rad(90.0)))
    target_pose.position.x = -0.3
    target_pose.position.y = 1.9
    target_pose.position.z = 1.2

    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.75)

    target_pose.position.x = Gaskets[rand][0]
    target_pose.position.y = Gaskets[rand][1]
    target_pose.position.z = Gaskets[rand][2]

    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    Gaskets.pop(rand)

    gripper_prox(True)
    rospy.sleep(0.75)

    target_pose.position.x = -0.3
    target_pose.position.y = 1.9
    target_pose.position.z = 1.2

    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.75)

    group.clear_pose_targets()
    joint_values = group.get_current_joint_values()
    joint_values[1] = 0.0

    group.set_joint_value_target(joint_values)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.75)

    target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
    target_pose.position.x = 1.1
    target_pose.position.y = 1.7
    target_pose.position.z = 0.98

    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    gripper_prox(False)
    rospy.sleep(0.75)

    target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
    target_pose.position.x = 1.1
    target_pose.position.y = 1.7
    target_pose.position.z = 1.2

    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.75)

    group.clear_pose_targets()
    joint_values = group.get_current_joint_values()
    joint_values[1] = 3.14

    group.set_joint_value_target(joint_values)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.75)
