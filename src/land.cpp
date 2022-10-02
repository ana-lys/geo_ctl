#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "land");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("land");
  std_srvs::SetBool::Request land_rq;
  std_srvs::SetBool::Response land_rs;
  if(argc == 2)
  land_rq.data = std::atoi(argv[1]);
  else land_rq.data = true;
  bool success = client.call(land_rq,land_rs);
  if(success && argv[1] == "0") 
  ROS_INFO_STREAM("Marker not found");
  else if (success)
  ROS_INFO_STREAM("Landing");
  else 
  ROS_INFO_STREAM("RIP DRONE 6/2022"); 
  double ta =3.0/100;
  for (int i = 0; i <100 ;i++){
  double tc = pow((0.8 * sinc (2.0 * ta*i -3.0)-0.025),2);
  ROS_INFO_STREAM(i*ta<<" "<<tc); }
  return 0;
}