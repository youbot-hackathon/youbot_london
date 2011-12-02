#!/usr/bin/python
import roslib; roslib.load_manifest('youbot_scenarios')

import sys
import math
import rospy
import smach
import smach_ros
import actionlib
import move_base_msgs.msg
import tower_of_hanoi_sdk.srv
import tower_of_hanoi_sdk.msg
import tf
import geometry_msgs.msg
import arm_configuration


world_frame = "/openni_rgb_optical_frame"
tf_listener = 0



class approach_pose(smach.State):

	def __init__(self, pose = ""):

		smach.State.__init__(
			self,
			outcomes=['success', 'failed'],
			input_keys=['pose', 'message'],
			output_keys=['pose', 'message'])

		self.pose = pose
		self.move_base = actionlib.SimpleActionClient("move_base", move_base_msgs.msg.MoveBaseAction)


	def execute(self, userdata):
		# determine target position
		if self.pose != "":
			pose = self.pose
		elif type(userdata.pose) is str:
			pose = userdata.pose
		elif type(userdata.pose) is list:
			pose = []
			pose.append(userdata.pose[0])
			pose.append(userdata.pose[1])
			pose.append(userdata.pose[2])
		else: # this should never happen
			userdata.message = []
			userdata.message.append(5)
			userdata.message.append("Invalid userdata 'pose'")
			userdata.message.append(userdata.pose)
			return 'failed'
		
		self.move_base.wait_for_server()
		
		goal = move_base_msgs.msg.MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = pose[0]
		goal.target_pose.pose.position.y = pose[1]
		goal.target_pose.pose.orientation.w = pose[2]
		
		self.move_base.send_goal(goal)
		self.move_base.wait_for_result(rospy.Duration(20.0))
		
		if (self.move_base.get_state() == actionlib.simple_action_client.GoalStatus.SUCCEEDED):
			return "success"
		else:
			return "failed"



class find_object(smach.State):
	def __init__(self, color):
		smach.State.__init__(self, outcomes=['success', 'no_object'], 
									output_keys=['grasp_position'])
		
		global tf_listener
		self.move_arm = arm_configuration.ArmConfiguration(tf_listener)
		self.object_list_srv = rospy.ServiceProxy('/youbot_3d_world_model/getSceneObjects', tower_of_hanoi_sdk.srv.GetSceneObjects)
		
		self.color = color
	
	def execute(self, userdata):
		self.move_arm.moveToConfiguration("zeroposition")
		self.move_arm.moveToConfiguration("kinect_left_init")
		self.move_arm.moveToConfiguration("kinect_left")
		
		rospy.wait_for_service('/youbot_3d_world_model/getSceneObjects', 10)
		attrib = tower_of_hanoi_sdk.msg.Attribute()
		attrib.key = "shapeType"
		attrib.value = "Box"
		attrib.key = "color"
		attrib.value = self.color
		req = tower_of_hanoi_sdk.srv.GetSceneObjectsRequest()
		req.attributes.append(attrib)
		
		rospy.sleep(3.0)
		
		
		# do the measurements that are required for smoothing
		measurements = []
		length = 10
		max_id = 0
		for k in range(length):
			resp = self.object_list_srv(req)
			
			measurements.append(resp)
			
			# find the maximum id to prepare the sum array
			for i in range(len(resp.results)):
				if (resp.results[i].id > max_id):
					max_id = resp.results[i].id
			
			rospy.sleep(0.2)
		
		
		coord = [[0, 0, 0] for i in range(max_id)]
		
		
		# sum the coordinates for each id in each measurement
		for k in range(length):
			elements = len(measurements[k].results)
			
			for i in range(elements):
				id = measurements[k].results[i].id - 1
				t = measurements[k].results[i].transform.transform.translation
				coord[id][0] += t.x / length
				coord[id][1] += t.y / length
				coord[id][2] += t.z / length
		
		
		# find the nearest object
		object_found = False
		min_dist = 1000.0
		min_index = -1
		for k in range(max_id):
			x = coord[k][0]
			y = coord[k][1]
			z = coord[k][2]
			
			dist = math.sqrt(x * x + y * y + z * z)
			
			if ((dist <= min_dist) and (dist >= 0.6)):
				min_dist = dist
				min_index = k
				object_found = True
		
		if (not object_found):
			return "no_object"
		
		global tf_listener
		
		pose = geometry_msgs.msg.PoseStamped()
		pose.header.frame_id = "/openni_rgb_optical_frame"
		pose.header.stamp = rospy.Time.now() - rospy.Duration(0.5)
		pose.pose.position.x = coord[min_index][0]
		pose.pose.position.y = coord[min_index][1]
		pose.pose.position.z = coord[min_index][2]
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		
		base_pose = tf_listener.transformPose("base_link", pose)
		userdata.grasp_position = base_pose
		
		print base_pose
		
		self.move_arm.moveToConfiguration("kinect_left_init")
		self.move_arm.moveToConfiguration("zeroposition")
		
		return "success"



class find_station(smach.State):

	def __init__(self, station):
		smach.State.__init__(self, outcomes=['success', 'no_object'])
		self.object_list_srv = rospy.ServiceProxy('/youbot_3d_world_model/getSceneObjects', tower_of_hanoi_sdk.srv.GetSceneObjects)
		self.station = station
		
		global tf_listener
		self.move_arm = arm_configuration.ArmConfiguration(tf_listener)

	def execute(self, userdata):
		self.move_arm.moveToConfiguration("zeroposition")
		self.move_arm.moveToConfiguration("kinect_left_init")
		self.move_arm.moveToConfiguration("kinect_left")
		
		rospy.wait_for_service('/youbot_3d_world_model/getSceneObjects', 10)
		attrib = tower_of_hanoi_sdk.msg.Attribute()
		attrib.key = "name"
		attrib.value = self.station
		req = tower_of_hanoi_sdk.srv.GetSceneObjectsRequest()
		req.attributes.append(attrib)
		
		resp = self.object_list_srv(req)
		
		if (len(resp.results) > 0):
			print resp.results[0]
			
			global tf_listener
			
			pose = geometry_msgs.msg.PoseStamped()
			pose.header = resp.results[0].transform.header
			pose.pose.position = resp.results[0].transform.transform.translation
			pose.pose.orientation = resp.results[0].transform.transform.rotation
			world_pose = tf_listener.transformPose(world_frame, pose)
			
			if (self.station == "start"):
				userdata.start = world_pose
			elif (self.station == "auxiliary"):
				userdata.auxiliary = world_pose
			elif (self.station == "goal"):
				userdata.goal = world_pose
			
			result = 'success'			
		else:
			result = 'no_object'
		
		self.move_arm.moveToConfiguration("kinect_left_init")
		self.move_arm.moveToConfiguration("zeroposition")
		
		return result



class grasp_object(smach.State):
	
	def __init__(self):
		smach.State.__init__(self, outcomes=['success','failed'], input_keys=['grasp_position'])
		
		global tf_listener
		self.move_arm = arm_configuration.ArmConfiguration(tf_listener)
	
	
	def execute(self, userdata):
		self.move_arm.moveGripperOpen()
		self.move_arm.moveToConfiguration("zeroposition")
		rospy.sleep(2.0)
		
		point = userdata.grasp_position.pose.position
		target_pose = self.move_arm._createPose(point.x + 0.01, point.y + 0.015, point.z + 0.06, 0, math.pi, 0)
		self.move_arm.moveToPose(target_pose)
		
		rospy.sleep(1.0)
		
		self.move_arm.moveGripperClose()
		
		rospy.sleep(2.0)
		
		self.move_arm.moveToConfiguration("zeroposition")
		
		return "success"


class release_object(smach.State):
	
	def __init__(self, position):
		smach.State.__init__(self, outcomes=['success', 'failed'])
		self.position = position
		
		global tf_listener
		self.move_arm = arm_configuration.ArmConfiguration(tf_listener)
	
	
	def execute(self, userdata):
		self.move_arm.moveToConfiguration("zeroposition")
		self.move_arm.moveToConfiguration("pregrasp_back_init")
		self.move_arm.moveToConfiguration("pregrasp_back")
		
		if (self.position == "front"):
			target_pose = self.move_arm._createPose(0.033 + 0.024 - 0.235, 0.0, 0.14, 0, -math.pi + 0.2, 0, "arm_link_0")
		else:
			target_pose = self.move_arm._createPose(0.033 + 0.024 - 0.28, 0.0, 0.14, 0, -math.pi + 0.3, 0, "arm_link_0")
		self.move_arm.moveToPose(target_pose)
		
		rospy.sleep(1.0)
		self.move_arm.moveGripperOpen()
		rospy.sleep(2.0)
		
		return "success"
		


class find_station(smach.State):

	def __init__(self, station):
		smach.State.__init__(self, outcomes=['success', 'no_object'])
		self.object_list_srv = rospy.ServiceProxy('/youbot_3d_world_model/getSceneObjects', tower_of_hanoi_sdk.srv.GetSceneObjects)
		self.station = station
		
		global tf_listener
		self.move_arm = arm_configuration.ArmConfiguration(tf_listener)



def main():
	rospy.init_node('youbot_london')
	
	global tf_listener
	tf_listener = tf.TransformListener()

	# Create a SMACH state machine
	sm = smach.StateMachine(outcomes=['overall_success', 'overall_failed'])
	
	sm.userdata.start = 0
	sm.userdata.auxiliary = 0
	sm.userdata.goal = 0
	sm.userdata.grasp_pose = 0
	
	with sm:
		#smach.StateMachine.add('approach_start', approach_pose([0, 0, 0]), transitions={'success':'overall_success', 'failed':'overall_success'})
		smach.StateMachine.add('identify_object_red', find_object("red"), transitions={'success':'grasp_object_red', 'no_object':'overall_failed'})
		smach.StateMachine.add('grasp_object_red', grasp_object(), transitions={'success':'release_object_red', 'failed':'overall_failed'})
		smach.StateMachine.add('release_object_red', release_object("front"), transitions={'success':'identify_object_yellow_1', 'failed':'overall_failed'})
		
		smach.StateMachine.add('identify_object_yellow_1', find_object("yellow"), transitions={'success':'grasp_object_yellow_1', 'no_object':'overall_failed'})
		smach.StateMachine.add('grasp_object_yellow_1', grasp_object(), transitions={'success':'release_object_yellow_1', 'failed':'overall_failed'})
		smach.StateMachine.add('release_object_yellow_1', release_object("rear"), transitions={'success':'identify_object_red', 'failed':'overall_failed'})
		
		#smach.StateMachine.add('identify_object_yellow_2', find_object("yellow"), transitions={'success':'grasp_object_yellow_2', 'no_object':'overall_failed'})
		#smach.StateMachine.add('grasp_object_yellow_2', grasp_object(), transitions={'success':'identify_object_yellow_1', 'failed':'overall_failed'})
		
		#smach.StateMachine.add('find_station_start', find_station("start"), transitions={'success':'find_station_auxiliary', 'no_object':'overall_success'})
		#smach.StateMachine.add('find_station_auxiliary', find_station("auxiliary"), transitions={'success':'find_station_goal', 'no_object':'overall_success'})
		#smach.StateMachine.add('find_station_goal', find_station("goal"), transitions={'success':'overall_success', 'no_object':'overall_success'})



									
	# Create and start the introspection server
	sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
	sis.start()

	# Execute the state machine
	rospy.sleep(5)
	outcome = sm.execute()
	
	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()


if __name__ == '__main__':
	main()