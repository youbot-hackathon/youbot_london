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
import actionlib
import control_msgs.msg
import trajectory_msgs.msg

from brics_actuator.msg import JointPositions, JointValue, Poison


class ArmConfiguration:
    def __init__(self):
        
        self.joint_names = ["arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"]
        self.configuration = [0, 0, 0, 0, 0]
        self.received_state = False

        rospy.Subscriber('/joint_states', sensor_msgs.msg.JointState, self._joint_states_callback)

        rospy.loginfo("Waiting for 'get_constraint_aware_ik' service")
        rospy.wait_for_service('/youbot_arm_kinematics/get_constraint_aware_ik')
        self.ciks = rospy.ServiceProxy('/youbot_arm_kinematics/get_constraint_aware_ik', kinematics_msgs.srv.GetConstraintAwarePositionIK)
        rospy.loginfo("Service 'get_constraint_aware_ik' is ready")

        
        self.gripper_publisher = rospy.Publisher('/arm_1/gripper_controller/position_command', JointPositions)
        
        self.armclient = actionlib.SimpleActionClient("/arm_1/arm_controller/joint_trajectory_action", 
                                          control_msgs.msg.FollowJointTrajectoryAction)
        print "waiting for action server"
        self.armclient.wait_for_server()
    
        
    #callback function: when a joint_states message arrives, save the values
    def _joint_states_callback(self, msg):
        for k in range(5):
            for i in range(len(msg.name)):
                joint_name = "arm_joint_" + str(k + 1)
                if(msg.name[i] == joint_name):
                    self.configuration[k] = msg.position[i]
        self.received_state = True


    def _call_constraint_aware_ik_solver(self, goal_pose):
        while (not self.received_state):
            time.sleep(0.1)
        req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()
        req.timeout = rospy.Duration(0.5)
        req.ik_request.ik_link_name = "arm_link_5"
        req.ik_request.ik_seed_state.joint_state.name = self.joint_names
        req.ik_request.ik_seed_state.joint_state.position = self.configuration
        req.ik_request.pose_stamped = goal_pose
        try:
            resp = self.ciks(req)
        except rospy.ServiceException, e:
            rospy.logerr("Service did not process request: %s", str(e))
        return (resp.solution.joint_state.position, resp.error_code.val == motion_planning_msgs.msg.ArmNavigationErrorCodes.SUCCESS)
        
    def _createPose(self, x, y, z, roll, pitch, yaw):
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
       
    def moveToPosition(self, joint_config):
            goal = control_msgs.msg.FollowJointTrajectoryGoal()
        
            goal.trajectory.joint_names = self.joint_names
        
            jtp = trajectory_msgs.msg.JointTrajectoryPoint()
                
            for i in range(5):
                jtp.positions.append(joint_config[i])
            
            goal.trajectory.points.append(jtp)
            print "sending goal"
            self.armclient.send_goal(goal)
        
            self.armclient.wait_for_result()
             
    def moveToPose(self, pose):
        print "gotoLocation: ", pose.pose.position.x, ", " , pose.pose.position.y, ", ", pose.pose.position.z
        
        (conf, success) = self._call_constraint_aware_ik_solver(pose)
        if (success):
            # publish solution directly as joint positions
            self.moveToPosition(conf)
            
            return True
        else:
            print("IK solver didn't find a solution")
            return False
    
    
    def moveToLocation(self, x,y,z, roll, pitch, yaw):
        return self.moveToPose(self._createPose(x,y,z, roll, pitch, yaw));
    
    def moveToConfiguration(self, arm_configuration_name):
        configname = "/arm_configurations/" + arm_configuration_name
        print "moveto ", configname
        if rospy.has_param(configname):
            config = rospy.get_param(configname);
            self.moveToPosition(config)
            return True
        else:
            print "Configuration ", arm_configuration_name, " does not exist"
            return False
        
    
    def _createGripperJointPositions(self, left, right):
        jp = JointPositions()

        jv1 = JointValue()
        jv1.joint_uri = "gripper_finger_joint_l"
        jv1.value = left
        jv1.unit = "m"

        jv2 = JointValue()
        jv2.joint_uri = "gripper_finger_joint_r"
        jv2.value = right
        jv2.unit = "m"

        p = Poison()
        jp.poisonStamp = p

        jp.positions = [jv1, jv2] #list

        return jp
    
    def _createGripperJointPositionsSym(self, value):
        return self._createGripperJointPositions(value, value)
        
    def moveGripperOpen(self):
        print "moveGripperOpen"
        jp = self._createGripperJointPositionsSym(0.0115)
        self.gripper_publisher.publish(jp)
     
    def moveGripperClose(self):
        print "moveGripperClose"
        jp = self._createGripperJointPositionsSym(0.00)
        self.gripper_publisher.publish(jp)
        
    '''
    percentave=0.0 => gripper close
    percentage=1.0 => gripper open  
    '''
    def moveGripper(self, percentage):
        print "moveGripper(", percentage,")"
        jp = self._createGripperJointPositionsSym(0.0115 * percentage)
        self.gripper_publisher.publish(jp)   
        
    def moveGripperOpeningDistance(self, distance):
        print "moveGripperOpeningDistance(", distance,")"
        jp = self._createGripperJointPositionsSym(distance / 2.0)
        self.gripper_publisher.publish(jp)       
    
    def moveGripperDirect(self, left, right):
        print "moveGripperOpeningDistance(", left,",", right,")"
        jp = self._createGripperJointPositions(left, right)
        self.gripper_publisher.publish(jp) 
