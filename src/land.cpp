#include "ros/ros.h"
#include <cstdlib>
#include "geometric_controller/geometric_controller.h"
#include "gazebo_msgs/GetLinkState.h"
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <controller_msgs/FlatTarget.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/GPSRAW.h>
// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "land");
//   ros::NodeHandle n;
//   ros::Publisher load_ground_truth,base_ground_truth;
//   load_ground_truth = n.advertise<nav_msgs::Odometry>("load_ground_truth", 1 );
//   base_ground_truth = n.advertise<nav_msgs::Odometry>("base_ground_truth", 1 );
//   while(true){
//   ros::service::waitForService("gazebo/get_link_state");
//   ros::ServiceClient client = n.serviceClient<gazebo_msgs::GetLinkState>("gazebo/get_link_state");
//   gazebo_msgs::GetLinkState::Request request_load,request_body;
//   gazebo_msgs::GetLinkState::Response response_load,response_body;
//   request_load.link_name = "solo::load_link";
//   bool success = client.call(request_load, response_load);
//   nav_msgs::Odometry load_gc ;
//   load_gc.pose.pose.position.x = response_load.link_state.pose.position.x;
//   load_gc.pose.pose.position.y = response_load.link_state.pose.position.y;
//   load_gc.pose.pose.position.z = response_load.link_state.pose.position.z;
  
//   load_gc.twist.twist.linear.x = response_load.link_state.twist.linear.x;
//   load_gc.twist.twist.linear.y = response_load.link_state.twist.linear.y;
//   load_gc.twist.twist.linear.z = response_load.link_state.twist.linear.z;
//   load_ground_truth.publish(load_gc); 
  
//   request_body.link_name = "solo::base_link";
//   success = client.call(request_body, response_body);
//   load_gc.pose.pose.position.x = response_body.link_state.pose.position.x;
//   load_gc.pose.pose.position.y = response_body.link_state.pose.position.y;
//   load_gc.pose.pose.position.z = response_body.link_state.pose.position.z;
  
//   load_gc.twist.twist.linear.x = response_body.link_state.twist.linear.x;
//   load_gc.twist.twist.linear.y = response_body.link_state.twist.linear.y;
//   load_gc.twist.twist.linear.z = response_body.link_state.twist.linear.z;
//   base_ground_truth.publish(load_gc);
//   loop_rate.sleep();
//   ROS_INFO_STREAM ("pos"<<response_load.link_state.pose.position.x - response_body.link_state.pose.position.x <<" "<<response_load.link_state.pose.position.y -response_body.link_state.pose.position.y<<" "<<response_load.link_state.pose.position.z-response_body.link_state.pose.position.z);
//   }// ROS_INFO_STREAM ("vel"<<response_load.link_state.twist.linear.x<<" "<<response_load.link_state.twist.linear.y<<" "<<response_load.link_state.twist.linear.z);}
//   return 0;
// }

string mavPos = " ";
string mavVel = " ";
string mavAtt = " ";
string Thrust = " 0 0 0 ";
string state = " ";
string battery = " ";
string gpsStatus =" ";
string green   ="\033[;32m";
string red     ="\033[1;31m";
string yellow  ="\033[1;33m";
string blue    ="\033[;34m";
string normal  ="\033[0m";
string prefix = " ";
string suffix = " ";
float voltage = 0;
int pose_status = 0;
void odomCallback(const nav_msgs::Odometry &odomMsg){
if(odomMsg.pose.pose.position.z > 15.0 || fabs(odomMsg.pose.pose.position.x)> 12.0 || fabs(odomMsg.pose.pose.position.y)> 12.0) pose_status = 3;
else if(odomMsg.pose.pose.position.z > 10.0 || fabs(odomMsg.pose.pose.position.x)> 10.0 || fabs(odomMsg.pose.pose.position.y)> 10.0) pose_status = 2;
else if(odomMsg.pose.pose.position.z > 5.0 || fabs(odomMsg.pose.pose.position.x)> 7.0 || fabs(odomMsg.pose.pose.position.y)> 7.0) pose_status = 1;
else pose_status = 0; 
if (pose_status == 0) {prefix = ""; suffix ="";}
else if(pose_status == 1) {prefix = green ; suffix = normal;}
else if(pose_status == 2) {prefix = yellow ; suffix = normal;}
else if(pose_status == 3) {prefix = red ; suffix = normal;}

 mavPos =prefix +" x: "+ to_string(odomMsg.pose.pose.position.x) + " y: "+ to_string(odomMsg.pose.pose.position.y) + " z: "+ to_string(odomMsg.pose.pose.position.z) +suffix;
 mavAtt =" x: "+ to_string(odomMsg.pose.pose.orientation.x) + " y: "+ to_string(odomMsg.pose.pose.orientation.y) + " z: "+ to_string(odomMsg.pose.pose.orientation.z) + " w: "+ to_string(odomMsg.pose.pose.orientation.w);

 if(fabs(odomMsg.twist.twist.linear.z )> 3.0 || fabs(odomMsg.twist.twist.linear.x)> 3.0 || fabs(odomMsg.twist.twist.linear.y)> 3.0) pose_status = 3;
else if(fabs(odomMsg.twist.twist.linear.z) > 2.0 || fabs(odomMsg.twist.twist.linear.x)> 2.0 || fabs(odomMsg.twist.twist.linear.y)> 2.0) pose_status = 2;
else if(fabs(odomMsg.twist.twist.linear.z )> 1.0 || fabs(odomMsg.twist.twist.linear.x)> 1.0 || fabs(odomMsg.twist.twist.linear.y)> 1.0) pose_status = 1;
else pose_status = 0; 
if (pose_status == 0) {prefix = ""; suffix ="";}
else if(pose_status == 1) {prefix = green ; suffix = normal;}
else if(pose_status == 2) {prefix = yellow ; suffix = normal;}
else if(pose_status == 3) {prefix = red ; suffix = normal;}

 mavVel =prefix +" x: "+ to_string(odomMsg.twist.twist.linear.x) + " y: "+ to_string(odomMsg.twist.twist.linear.y) + " z: "+ to_string(odomMsg.twist.twist.linear.z) +suffix;
 
}

void rawattCallback(const mavros_msgs::AttitudeTarget &attmsg){
  Thrust =" Throttle := "+ to_string(attmsg.thrust) + " wX: "+ to_string(attmsg.body_rate.x) + " wY: "+ to_string(attmsg.body_rate.y) + " wZ: "+ to_string(attmsg.body_rate.z);
}

void stateCallback(const mavros_msgs::State &msg) { 
  state = "Arm status "+ to_string(msg.armed) + " Flightmode " + msg.mode;
 }
void batteryCallback(const sensor_msgs::BatteryState &msg){
  voltage = msg.voltage;
  if(voltage > 16.0) pose_status = 0;
  else if(voltage > 15.5) pose_status = 1;
  else if(voltage > 15.3) pose_status = 2;
  else if(voltage > 15.18) pose_status = 3;
  
  if (pose_status == 0) {prefix = ""; suffix ="";}
 else if(pose_status == 1) {prefix = green ; suffix = normal;}
 else if(pose_status == 2) {prefix = yellow ; suffix = normal;}
 else if(pose_status == 3) {prefix = red ; suffix = normal;}

  battery = prefix + "Voltage " +to_string(voltage) +" less than "+ to_string( 100 - 5 * int((16.80 - msg.voltage)/0.15)) +" % " + suffix ;
}
void gpsStatusCallback(const mavros_msgs::GPSRAW &msg){

  gpsStatus= " H_acc " +to_string(int(msg.h_acc)) +" V_acc "+ to_string(int(msg.v_acc)) +" Vel_acc " + to_string(int(msg.vel_acc))+" Sat_num" +to_string(msg.satellites_visible) ;
}
int main(int argc, char **argv) {
  
  ros::init(argc, argv, "land");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  ros::Subscriber odomSub_ = n.subscribe("mavros/local_position/odom", 1, &odomCallback,
                               ros::TransportHints().tcpNoDelay());
  ros::Subscriber raw_attSub_ = n.subscribe("/mavros/setpoint_raw/attitude", 1, &rawattCallback,ros::TransportHints().tcpNoDelay());                 
  ros::Subscriber mav_stateSub_ =n.subscribe("mavros/state", 1, &stateCallback, ros::TransportHints().tcpNoDelay());  
  ros::Subscriber batterySub_ = n.subscribe("mavros/battery", 1, &batteryCallback,  ros::TransportHints().tcpNoDelay());
  ros::Subscriber gpsSub_ = n.subscribe("/mavros/gpsstatus/gps1/raw", 1, &gpsStatusCallback,  ros::TransportHints().tcpNoDelay());

   while(ros::ok()){
   loop_rate.sleep();
   cout << "Position " << mavPos << endl;
   cout << "Velocity " << mavVel << endl;
   cout << "Orientation " << mavAtt << endl;
   cout << "Raw_setpoint" << Thrust << endl;
   cout << "Mav_State " << state << endl;
   cout << "Battery " << battery << endl;
   cout << "GPS "<< gpsStatus << endl;
   cout << "\033[2J\033[1;1H";
   ros::spinOnce();
   }
   
   return 0;
 }