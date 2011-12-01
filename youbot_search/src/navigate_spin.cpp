#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;
ros::Subscriber sub;
move_base_msgs::MoveBaseGoal goal;
ros::Publisher cmd_pub;

double angz;
double angle_difference;
double turn_speed;
bool read_turn;

bool nextgoal(std_srvs::Empty::Request  &req,
         std_srvs::Empty::Response &res)
{
	ros::Rate loop_rate(1);
	float old_ang = angz;
	while (ros::ok())
	  {

	    geometry_msgs::Twist twist;

	    twist.linear.x = 0;
	    twist.linear.y = 0;
	    twist.linear.z = 0;

	    twist.angular.x = 0;
	    twist.angular.y = 0;
	    twist.angular.z = turn_speed;

	    cmd_pub.publish(twist);

	     loop_rate.sleep();
 	    if (read_turn) 
  		{if ((fabs(old_ang-angz)  > 0.1) ) break;}
	    if (fabs(old_ang-angz) > 1) read_turn = true;
   

	  }
	return true;
}

void odom_callback(const nav_msgs::Odometry::ConstPtr& data)
{
 	angz = data->pose.pose.orientation.w;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "navigate_search_spin");
  ac = new MoveBaseClient("move_base", true);
  angz = 0.0;
  read_turn = false;

  while(!ac->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  goal.target_pose.header.frame_id = "odom";

   // get parameters
  ros::NodeHandle nh("~");
  
  nh.param<double>("angle_difference", angle_difference, 0.01);
  nh.param<double>("turn_speed", turn_speed, 5);



  ros::NodeHandle n;
 
  ros::ServiceServer service = n.advertiseService("navigate_search_spin", nextgoal);

  sub = n.subscribe("/odom", 1000, odom_callback);
  cmd_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
  ROS_INFO("Ready to navigate to next init goal");
  ros::spin();

  return 0;
}


