#include <cstdlib>
#include <iostream>
#include <string>
#include "ros/ros.h"
#include "moveit_msgs/MoveGroupActionResult.h"


void chatterCallback(const moveit_msgs::MoveGroupActionResult::ConstPtr& msg)
{
int axesUsed = 5;

double startArray[axesUsed];

double viaPointArray[msg->result.planned_trajectory.joint_trajectory.points.size()][3][axesUsed];

 if(msg->result.trajectory_start.joint_state.position.size() != 0)
{
	for(int i = 0; i < msg->result.trajectory_start.joint_state.position.size(); i++)//this loop goes through all values of the trajectory points
	{
		startArray[i] = msg->result.trajectory_start.joint_state.position[i];
	}
} 
 
if(msg->result.planned_trajectory.joint_trajectory.points.size() != 0)
{
	for(int i = 0; i < msg->result.planned_trajectory.joint_trajectory.points.size(); i++)//this loop goes through all values of the trajectory points
	{
		for(int j = 0; j < 5; j++)// this loop goes through all joints
		{
			viaPointArray[i][0][j] = msg->result.planned_trajectory.joint_trajectory.points[i].positions[j];
		}	
	}
}

 ROS_INFO("I heard: ");
ros::NodeHandle n;
  n.setParam("/axesUsed", axesUsed);
  n.setParam("/startArray", startArray);

  int viaPointSize = msg->result.planned_trajectory.joint_trajectory.points.size();
  n.setParam("/viaPointSize", viaPointSize);
  n.setParam("/viaPointArray", viaPointArray);
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "trajectory_listener");

  ros::NodeHandle n;

  
  ros::Subscriber sub = n.subscribe("/move_group/result", 1000, chatterCallback);	


  ros::spin();

  return 0;
}
