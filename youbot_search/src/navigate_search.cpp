#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include "nav_msgs/Odometry.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int goalcount = 0;
double max_x, max_y;
ros::Subscriber sub;
MoveBaseClient *ac;
  move_base_msgs::MoveBaseGoal goal;
double angz;

bool nextgoal(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
	float random = ((float) rand()) / (float) RAND_MAX;
	random = random * 2 - 1;
	ROS_INFO("random value = %f", random);
	goal.target_pose.pose.position.x = random * max_x;
	goal.target_pose.pose.position.y = random * max_y;
	goal.target_pose.pose.orientation.w = random*max_x;

	ac->sendGoal(goal);
	ac->waitForResult();

	if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Reached goal");
	} else {
    		ROS_INFO("The base failed to reach goal");
	}
	goalcount++;	
	return true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
{
 	angz = data->pose.pose.orientation.w;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigate_search");
  
  ac = new MoveBaseClient("move_base", true);

  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  goal.target_pose.header.frame_id = "odom";
  
  // get parameters
  ros::NodeHandle nh("~");
  nh.param<double>("max_x", max_x, 2.0);
  nh.param<double>("max_y", max_y, 2.0);
	

  ros::NodeHandle n;
 sub = n.subscribe("/odom", 1000, odom_callback);
  ros::ServiceServer service = n.advertiseService("navigate_search", nextgoal);
  ROS_INFO("Ready to navigate to next init goal");
  ros::spin();

  return 0;
}


