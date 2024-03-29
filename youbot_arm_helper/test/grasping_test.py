#!/usr/bin/env python
import roslib; roslib.load_manifest('youbot_arm_helper')

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

from brics_actuator.msg import JointPositions, JointValue, Poison

import youbot_arm_helper.arm_configuration
from youbot_arm_helper.arm_configuration import ArmConfiguration
from tf import transformations

def createPose(x, y, z, roll, pitch, yaw):
        pose = geometry_msgs.msg.PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        pose.header.frame_id = "/arm_link_0"
        pose.header.stamp = rospy.Time.now()  
        return pose
    
def poseFrontPreGrasp():
        x = 0.25
        y = 0
        z = 0.10
        roll = 0
        pitch = math.pi
        yaw = 0
        return createPose(x, y, z, roll, pitch, yaw)
 
 
if __name__ == "__main__":
    rospy.init_node('youbot_ik_solver_test')
    time.sleep(0.5)
    
    an = ArmConfiguration()

    #front center
    targetPose_front_01 = an._createPose(0.27, 0.0, 0.05, 0, math.pi, math.pi / 2)
    
    #front left
    targetPose_front_02 = an._createPose(0.27, 0.08, 0.05, 0, math.pi, math.pi / 2)
    
    #front right
    targetPose_front_03 = an._createPose(0.27, -0.08, 0.05, 0, math.pi, math.pi / 2)
    
    
    #end begin
    targetPose_back_01 = an._createPose(0.033 + 0.024 - 0.235, 0.0, 0.105, 0, -math.pi + 0.2, 0)
    
    #end middle
    targetPose_back_02 = an._createPose(0.033 + 0.024 - 0.28, 0.0, 0.105, 0, -math.pi + 0.3, 0)
    #targetPose_back = an._createPose(0.033 + 0.024 - 0.39, 0.00, 0.07, 0, -math.pi + 0.4, 0)
      
    #an.moveToConfiguration("pregrasp_back")
        
    #an.moveToPoseAndGraspFromTop(targetPose_back) 
        
    while (True):
       # an.moveToConfiguration("zeroposition")

        an.moveToConfiguration("pregrasp_front")
        
        an.moveToPoseAndGraspFromTop(targetPose_front_01)        
        
        an.moveToConfiguration("pregrasp_front_init")        
        #an.moveToConfiguration("zeroposition")
    
        an.moveToConfiguration("pregrasp_back_init")
        
        an.moveToConfiguration("pregrasp_back")
                
        an.moveToPoseAndReleaseFromTop(targetPose_back_01)
        
        
        an.moveToConfiguration("pregrasp_back")
        
        an.moveToConfiguration("pregrasp_back_init")
         
         
         
        an.moveToConfiguration("pregrasp_front")
        
        an.moveToPoseAndGraspFromTop(targetPose_front_02)        
        
        an.moveToConfiguration("pregrasp_front_init")        
        
        
        an.moveToConfiguration("pregrasp_back_init")
        
        an.moveToConfiguration("pregrasp_back")
                
        an.moveToPoseAndReleaseFromTop(targetPose_back_02)
        
        
        an.moveToConfiguration("pregrasp_back")
         
        an.moveToConfiguration("pregrasp_back_init")
        

        #an.moveToConfiguration("zeroposition")

        an.moveToConfiguration("pregrasp_front_init")
        
        an.moveToConfiguration("pregrasp_front")
        
        an.moveToPoseAndGraspFromTop(targetPose_front_03)
        
        
        
        an.moveToConfiguration("pregrasp_front")
        
        an.moveToPoseAndReleaseFromTop(targetPose_front_01)
        
        
        an.moveToConfiguration("pregrasp_back_init")
        
        an.moveToConfiguration("pregrasp_back")
                
        an.moveToPoseAndGraspFromTop(targetPose_back_02)
        
        an.moveToConfiguration("pregrasp_front")
        
        an.moveToPoseAndReleaseFromTop(targetPose_front_02)
        
        an.moveToConfiguration("pregrasp_front")
        
        
        an.moveToConfiguration("pregrasp_back_init")
        
        an.moveToConfiguration("pregrasp_back")
                
        an.moveToPoseAndGraspFromTop(targetPose_back_01)
        
        an.moveToConfiguration("pregrasp_front")
        
        an.moveToPoseAndReleaseFromTop(targetPose_front_03)
        
        an.moveToConfiguration("pregrasp_front")
        
