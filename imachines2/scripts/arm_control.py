#!/usr/bin/env python
import roslib; roslib.load_manifest('imachines2')

import rospy
import threading
import tf
import time
import math
import geometry_msgs.msg
import kinematics_msgs.srv
import kinematics_msgs.msg
import sensor_msgs.msg
import motion_planning_msgs.msg

import imachines2.msg

import youbot_arm_helper;

from brics_actuator.msg import JointPositions, JointValue, Poison

import youbot_arm_helper.arm_configuration
from youbot_arm_helper.arm_configuration import ArmConfiguration
from tf import transformations
import imachines2


## Offset between camara frame and target frame
offset_x = 0.17
offset_y = -0.005
offset_z = 0.05

class ArmControl:
    
    def __init__(self):
        self.arm = youbot_arm_helper.arm_configuration.ArmConfiguration()
        
        rospy.Subscriber("/cubelist", imachines2.msg.cubelist, self.cublistCB)
        
        

    def cublistCB(self, cubelist):
        print "Received ", len(cubelist.cubes) , " cubes "
        
        for i in range(len(cubelist.cubes)):
            c = imachines2.msg.cube
            c = cubelist.cubes[i]
            
            print "Color ", c.color
        
            if (c.color == "blue"):
                
                pose = self.defaultGraspPose()
                #pose = geometry_msgs.msg.PoseStamped()
                 
                pose.pose.position.x = c.centerx + offset_x
                pose.pose.position.y = c.centery + offset_y
                pose.pose.position.z = c.centerz + offset_z
                
                
                #self.arm.moveGripperOpen()
                
                
                self.arm.moveToPose(self.defaultGraspPose())
                
                print "Move arm to position "
                print pose
        
                #self.arm.moveToConfiguration("")
                self.arm.moveToPoseAndGraspFromTop(pose)
                
                self.arm.moveToConfiguration("pregrasp_front_init")
                
                self.arm.moveToConfiguration("pregrasp_back_init")

                self.arm.moveToConfiguration("pregrasp_back")
                      
                targetPose_back_01 = self.arm._createPose(0.033 + 0.024 - 0.235, 0.0, 0.105, 0, -math.pi + 0.2, 0)
              
                self.arm.moveToPoseAndReleaseFromTop(targetPose_back_01)
                
                self.arm.moveToConfiguration("pregrasp_back")
                
                self.arm.moveToConfiguration("pregrasp_back_init")
                 
                 
                self.arm.moveToConfiguration("pregrasp_front")
       
    def defaultGraspPose(self):
        x = 0.20 #0.25
        y = 0
        z = 0.15 # 0.10
        roll = 0
        pitch = math.pi + 0.1
        yaw = math.pi / 2
        return self.arm._createPose(x, y, z, roll, pitch, yaw)    


if __name__ == "__main__":
    rospy.init_node('imachines2_arm_control')
    
    arm = ArmControl()
    
    arm.arm.moveToConfiguration("pregrasp_front_init")
    arm.arm.moveToPose(arm.defaultGraspPose())
                           
    print "Waiting for cubelists"
    
    rospy.spin()
