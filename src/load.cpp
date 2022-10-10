#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"
#include "gazebo_msgs/GetLinkState.h"
#include <nav_msgs/Odometry.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "land");
  ros::NodeHandle n;
  ros::Publisher load_ground_truth,base_ground_truth;
  ros::Rate loop_rate(100);
  load_ground_truth = n.advertise<nav_msgs::Odometry>("load_ground_truth", 1 );
  base_ground_truth = n.advertise<nav_msgs::Odometry>("base_ground_truth", 1 );
  while(ros::ok()){
  ros::service::waitForService("gazebo/get_link_state");
  ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");
  gazebo_msgs::GetLinkState::Request request_load,request_body;
  gazebo_msgs::GetLinkState::Response response_load,response_body;
  request_load.link_name = "solo::load_link";
  bool success = client.call(request_load, response_load);
  nav_msgs::Odometry load_gc ;
  load_gc.pose.pose.position.x = response_load.link_state.pose.position.x;
  load_gc.pose.pose.position.y = response_load.link_state.pose.position.y;
  load_gc.pose.pose.position.z = response_load.link_state.pose.position.z;
  
  load_gc.twist.twist.linear.x = response_load.link_state.twist.linear.x;
  load_gc.twist.twist.linear.y = response_load.link_state.twist.linear.y;
  load_gc.twist.twist.linear.z = response_load.link_state.twist.linear.z;
  load_ground_truth.publish(load_gc); 
  
  request_body.link_name = "solo::base_link";
  success = client.call(request_body, response_body);
  load_gc.pose.pose.position.x = response_body.link_state.pose.position.x;
  load_gc.pose.pose.position.y = response_body.link_state.pose.position.y;
  load_gc.pose.pose.position.z = response_body.link_state.pose.position.z;
  
  load_gc.twist.twist.linear.x = response_body.link_state.twist.linear.x;
  load_gc.twist.twist.linear.y = response_body.link_state.twist.linear.y;
  load_gc.twist.twist.linear.z = response_body.link_state.twist.linear.z;
  base_ground_truth.publish(load_gc);
  loop_rate.sleep();
  ros::spinOnce();
  ROS_INFO_STREAM ("pos"<<response_load.link_state.pose.position.x - response_body.link_state.pose.position.x <<" "<<response_load.link_state.pose.position.y -response_body.link_state.pose.position.y<<" "<<response_load.link_state.pose.position.z-response_body.link_state.pose.position.z);
  }// ROS_INFO_STREAM ("vel"<<response_load.link_state.twist.linear.x<<" "<<response_load.link_state.twist.linear.y<<" "<<response_load.link_state.twist.linear.z);}
  return 0;
}