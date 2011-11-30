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
    
    #rospy.sleep(5.0)
    
    print "moveto config"
    #an.moveToConfiguration("zeroposition")
    
    
    an.moveGripperOpen()
    
    rospy.sleep(1.0)
    
    an.moveGripperClose()
    
    rospy.sleep(1.0)
    
    an.moveGripper(0.0)
    
    rospy.sleep(1.0)
    
    an.moveGripper(1.0)
    
    rospy.sleep(1.0)
    
    an.moveGripperDirect(0.0, 0.0115)
    
    rospy.sleep(0.0115)
    an.moveGripperDirect(0.0115, 0.0)
    
    rospy.sleep(0.0115)
    an.moveGripperOpeningDistance(0.0115*2)
    
    rospy.sleep(0.0115)
    an.moveGripperOpeningDistance(0.0)
    '''

    
    an.moveToConfiguration("pregrasp_front_init")
    an.moveToConfiguration("pregrasp_front")
    
    rospy.sleep(3.0)
    
    an.moveToConfiguration("zeroposition")
    
    an.moveToConfiguration("pregrasp_back_init")
    an.moveToConfiguration("pregrasp_back")
    rospy.sleep(3.0)
    
    an.moveToConfiguration("zeroposition")
    
    an.moveToConfiguration("pregrasp_left_init")
    an.moveToConfiguration("pregrasp_left")
    rospy.sleep(1.0)
    an.moveToConfiguration("pregrasp_left_init")
    
    an.moveToConfiguration("zeroposition")
    
    an.moveToConfiguration("pregrasp_right_init")
    an.moveToConfiguration("pregrasp_right")
    rospy.sleep(1.0)
    an.moveToConfiguration("pregrasp_right_init")
    
    an.moveToConfiguration("zeroposition")
    '''
    #an.moveToConfiguration("initposition")
    
    '''
    an.moveToConfiguration("kinect_right_init")
    an.moveToConfiguration("kinect_right")
    rospy.sleep(2.0)
    #an.moveToConfiguration("kinect_right_init")
  
    #an.moveToConfiguration("kinect_left_init")
    #an.moveToConfiguration("kinect_left")
   #rospy.sleep(1.0)
    an.moveToConfiguration("kinect_right_init")
    
    an.moveToConfiguration("kinect_left_init")
    an.moveToConfiguration("kinect_left")
    rospy.sleep(2.0)
    an.moveToConfiguration("kinect_left_init")
   # an.moveToConfiguration("zeroposition")
    
    '''
    '''
    x = 0.024 + 0.033
    y = 0
    z = 0.535
    roll = 0
    pitch = 0
    yaw = 0
    
    an.moveToLocation(x, y, z, roll, pitch, yaw)
    
    an.moveToPose(poseFrontPreGrasp())
    
    an.moveToConfiguration("initposition")
    '''
    #an.moveToLocation(pose)
    print "config reached"
    
    
    