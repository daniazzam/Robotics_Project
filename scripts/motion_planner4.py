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

import random
import numpy as np
pointer_of_gaskets = 0
pointer_of_gears = 0
pointer_of_pistons = 0



moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node("move_group_python_node")
rospy.wait_for_service('gripper/control')
rospy.loginfo('Pre service')
rospy.wait_for_service('Respawn_objects')
rospy.loginfo('Post Service')

gripper_prox = rospy.ServiceProxy('gripper/control', VacuumGripperControl)
respawn_request=rospy.ServiceProxy('Respawn_objects', Respawn)



robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")

trajectory_pub = rospy.Publisher("ariac/arm1/move_group/display_planned_path", DisplayTrajectory, queue_size = 10)

def get_object(coordinates):
    
    # joint_values = group.get_current_joint_values()
    # joint_values[0] = joint
    # group.set_joint_value_target(joint_values)
    # group.plan()
    # group.go(wait=True)

    target_pose = Pose()    
    group.clear_pose_targets()
    joint_values = group.get_current_joint_values()
    joint_values[1] = 3.14
    group.set_joint_value_target(joint_values)
    group.plan()
    group.go(wait=True)
    rospy.sleep(0.1)

    target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), np.deg2rad(90.0)))
    target_pose.position.x = coordinates[0]
    target_pose.position.y = coordinates[1]
    target_pose.position.z = 1.2
    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.1)

    # Go to target position
    target_pose.position.x = coordinates[0]
    target_pose.position.y = coordinates[1]
    target_pose.position.z = coordinates[2]
    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    # Activate Gripper
    gripper_prox(True)
    rospy.sleep(1)

    target_pose.position.x = coordinates[0]
    target_pose.position.y = coordinates[1]
    target_pose.position.z = 1.2
    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.1)

    group.clear_pose_targets()
    joint_values = group.get_current_joint_values()
    joint_values[1] = 0.0
    group.set_joint_value_target(joint_values)
    group.plan()
    group.go(wait=True)

    rospy.sleep(0.1)

    # Place Object on Conveyor Belt
    target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
    target_pose.position.x = 1.1
    target_pose.position.y = 1.7
    target_pose.position.z = 0.98
    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)
    gripper_prox(False)
    rospy.sleep(0.1)

    # Shift gripper upwards
    target_pose.orientation = Quaternion(*quaternion_from_euler(0.0, np.deg2rad(90.0), 0.0))
    target_pose.position.x = 1.1
    target_pose.position.y = 1.7
    target_pose.position.z = 1.2
    group.set_pose_target(target_pose)
    group.plan()
    group.go(wait=True)
    rospy.sleep(0.1)

    # Restore Default Position
    group.clear_pose_targets()
    joint_values = group.get_current_joint_values()
    joint_values[1] = 3.14
    group.set_joint_value_target(joint_values)
    group.plan()
    group.go(wait=True)
    rospy.sleep(0.1)

# get_object([-0.15, 2.07, 0.758])
# get_object([-0.15, 1.3, 0.749])
# get_object([-0.15, 0.53, 0.744])


gasket_1 = [-0.15, 2.07, 0.758]
gasket_2 = [-0.15, 1.76, 0.758]
gasket_3 = [-0.45, 2.07, 0.758]
gasket_4 = [-0.45, 1.76, 0.758]
Gaskets = [gasket_1, gasket_2, gasket_3, gasket_4]

gear_1 = [-0.15, 1.3, 0.75]
gear_2 = [-0.15, 1, 0.75]
gear_3 = [-0.45, 1.3, 0.75]
gear_4 = [-0.45, 1, 0.75]
Gears = [gear_1, gear_2, gear_3, gear_4]

piston_1 = [-0.15, 0.53, 0.75]
piston_2 = [-0.15, 0.23, 0.75]
piston_3 = [-0.45, 0.53, 0.75]
piston_4 = [-0.45, 0.23, 0.75]
Pistons = [piston_1, piston_2, piston_3, piston_4]

def main_loop(nb_gaskets, nb_gears, nb_pistons):
    global pointer_of_gaskets, pointer_of_gears, pointer_of_pistons
    
    i=0
    while i < nb_gaskets:
        get_object(Gaskets[pointer_of_gaskets])
        pointer_of_gaskets += 1
        if pointer_of_gaskets >= 1:
            rospy.loginfo('Fetna')
            pointer_of_gaskets = 0
            response = respawn_request('Gasket')
        i+=1
        
        if i >= nb_gaskets:
            break

main_loop(5, 0 , 0)
