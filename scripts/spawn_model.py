#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import SpawnModel
from gazebo_msgs.srv import DeleteModel
from geometry_msgs.msg import Pose, Quaternion
from robotics_project.srv import Respawn, RespawnResponse, RespawnRequest
from tf.transformations import quaternion_from_euler
import numpy as np

class Spawn:
	def __init__(self):
                rospy.init_node('spawn_model_node',log_level=rospy.INFO)

                rospy.Service('/ariac/arm1/Respawn_objects',Respawn, self.respawn_callback)
                rospy.wait_for_service('gazebo/spawn_sdf_model')
                rospy.wait_for_service('/gazebo/delete_model')
                delete_prox = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
                delete_prox("logical_camera_1")
                delete_prox("depth_camera_1")
                self.spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

                f_gasket = open('/opt/ros/melodic/share/osrf_gear/models/gasket_part_ariac/model.sdf','r')
                f_gear = open('/opt/ros/melodic/share/osrf_gear/models/gear_part_ariac/model.sdf', 'r')
                f_piston = open('/opt/ros/melodic/share/osrf_gear/models/piston_rod_part_ariac/model.sdf', 'r')

                self.sdf_gasket = f_gasket.read()
                self.sdf_gear = f_gear.read()
                self.sdf_piston = f_piston.read()
                self.counter = 0
                self.counter1= 0
                self.counter2= 0
                #self.spawn_model()
                #self.timer_callback(None)
                #rospy.Timer(rospy.Duration(1.0/0.1), self if (request.type=="Gasket"):
                self.spawn_on_bin_gasket()
                self.spawn_on_bin_gear()
                self.spawn_on_bin_piston()
                self.counter+=1
                self.counter1+=1
                self.counter2+=1

                rospy.spin()

        def respawn_callback(self,request):
                if (request.type=="Gasket"):
                        self.spawn_on_bin_gasket()
                        self.counter+=1

                if (request.type=="Gear"):
                        self.spawn_on_bin_gear()
                        self.counter1+=1

                if (request.type=="Piston"):
                        self.spawn_on_bin_piston()
                        self.counter2+=1

                response=RespawnResponse(True)
                return response

           

        def spawn_model(self):
                initial_pose = Pose()
                initial_pose.position.x = 0.5
                initial_pose.position.y = 0.0
                initial_pose.position.z = -1.0
                self.spawn_model_prox("Gasket_" + str(self.counter), self.sdf_gasket, "robotics_project", initial_pose, "laser_profiler_1")
                
        def spawn_on_bin_gasket(self):
                initial_pose = Pose()
                offset = 0.15
                initial_pose.position.x = offset
                initial_pose.position.y = offset
                initial_pose.position.z = 0.75
                self.spawn_model_prox("Gasket_" + str(self.counter), self.sdf_gasket, "robotics_project", initial_pose, "bin6")
                self.counter+=1
                initial_pose.position.x = -offset
                initial_pose.position.y = -offset
                self.spawn_model_prox("Gasket_" + str(self.counter), self.sdf_gasket, "robotics_project", initial_pose, "bin6")
                self.counter+=1
                initial_pose.position.x = -offset
                initial_pose.position.y = offset
                self.spawn_model_prox("Gasket_" + str(self.counter), self.sdf_gasket, "robotics_project", initial_pose, "bin6")
                self.counter+=1
                initial_pose.position.x = offset
                initial_pose.position.y = -offset
                self.spawn_model_prox("Gasket_" + str(self.counter), self.sdf_gasket, "robotics_project", initial_pose, "bin6")
                self.counter+=1

        def spawn_on_bin_gear(self):
                initial_pose = Pose()
                offset = 0.15
                initial_pose.position.x = offset
                initial_pose.position.y = offset
                initial_pose.position.z = 0.75
                self.spawn_model_prox("Gear_" + str(self.counter1), self.sdf_gear, "robotics_project", initial_pose, "bin5")
                self.counter1+=1
                initial_pose.position.x = -offset
                initial_pose.position.y = -offset
                self.spawn_model_prox("Gear_" + str(self.counter1), self.sdf_gear, "robotics_project", initial_pose, "bin5")
                self.counter1+=1
                initial_pose.position.x = -offset
                initial_pose.position.y = offset
                self.spawn_model_prox("Gear_" + str(self.counter1), self.sdf_gear, "robotics_project", initial_pose, "bin5")
                self.counter1+=1
                initial_pose.position.x = offset
                initial_pose.position.y = -offset
                self.spawn_model_prox("Gear_" + str(self.counter1), self.sdf_gear, "robotics_project", initial_pose, "bin5")
                self.counter1+=1

        def spawn_on_bin_piston(self):
                initial_pose = Pose()
                offset = 0.15
                initial_pose.position.x = offset
                initial_pose.position.y = offset
                initial_pose.position.z = 0.75
                initial_pose.orientation = Quaternion(*quaternion_from_euler(0.0, 0.0, np.deg2rad(90.0)))
                self.spawn_model_prox("Piston_" + str(self.counter2), self.sdf_piston, "robotics_project", initial_pose, "bin4")
                self.counter2+=1
                initial_pose.position.x = -offset
                initial_pose.position.y = -offset
                self.spawn_model_prox("Piston_" + str(self.counter2), self.sdf_piston, "robotics_project", initial_pose, "bin4")
                self.counter2+=1
                initial_pose.position.x = -offset
                initial_pose.position.y = offset
                self.spawn_model_prox("Piston_" + str(self.counter2), self.sdf_piston, "robotics_project", initial_pose, "bin4")
                self.counter2+=1
                initial_pose.position.x = offset
                initial_pose.position.y = -offset
                self.spawn_model_prox("Piston_" + str(self.counter2), self.sdf_piston, "robotics_project", initial_pose, "bin4")
                self.counter2+=1

if __name__ == '__main__':
        try:
                Spawn()
        except rospy.ROSInterruptException as e:
                pass


