/****************************************************************************
 *
 *   Copyright (c) 2018-2021 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Geometric Controller
 *
 * Geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "geometric_controller/geometric_controller.h"

using namespace Eigen;
using namespace std;
// Constructor
geometricCtrl::geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh),
      nh_private_(nh_private),
      fail_detec_(false),
      ctrl_enable_(true),
      landing_commanded_(false),
      feedthrough_enable_(false),
      node_state(WAITING_FOR_HOME_POSE) {
  referenceSub_ =
      nh_.subscribe("reference/setpoint", 1, &geometricCtrl::targetCallback, this, ros::TransportHints().tcpNoDelay());
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());
  ctrltriggerServ_ = nh_.advertiseService("trigger_rlcontroller", &geometricCtrl::ctrltriggerCallback, this);
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback,
                                   this);  // Define timer for constant loop rate
  statusloop_timer_ = nh_.createTimer(ros::Duration(1), &geometricCtrl::statusloopCallback,
                                      this);  // Define timer for constant loop rate
  gpsrawSub_= nh_.subscribe("mavros/global_position/global", 1, &geometricCtrl::gpsrawCallback, this,
                              ros::TransportHints().tcpNoDelay());
  batterySub_= nh_.subscribe("mavros/battery", 1, &geometricCtrl::batteryCallback, this,
                              ros::TransportHints().tcpNoDelay());
  global_poseSub_= nh_.subscribe("mavros/global_position/local", 1, &geometricCtrl::globalCallback, this,
                               ros::TransportHints().tcpNoDelay());
  load_ground_truthSub_= nh_.subscribe("/load_ground_truth", 1, &geometricCtrl::loadgroundtruth_callback, this,
                               ros::TransportHints().tcpNoDelay());      
  ImuPmarker_pub = nh_.advertise<visualization_msgs::Marker>( "/ImuP_marker", 0 );
  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);
  imuSub_ = nh_.subscribe("/mavros/imu/data",1,&geometricCtrl::imuCallback, this,
                              ros::TransportHints().tcpNoDelay());
  imu_physicalSub_ = nh_.subscribe("/Imu_load_stm32",1,&geometricCtrl::imuphysicalCallback, this,
                              ros::TransportHints().tcpNoDelay());                                    
  imuloadSub_ = nh_.subscribe("/imu_load",1,&geometricCtrl::imuloadCallback, this,
                              ros::TransportHints().tcpNoDelay());                            
  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, ERROR_QUATERNION);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("load_control_", load_control_, true);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_offset_, 0.1);    // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
  nh_private_.param<double>("Krp",  Krp_, 1.5);
  nh_private_.param<double>("Kyaw", Kyaw_, 2);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 5.0);
  nh_private_.param<double>("land_vel", Landing_velocity_, 1);
  Kpos_load << - 2.0 , - 2.0 , - 2.0;
  Kvel_load << - 4.0 , - 4.0 , - 4.0;
  Landing_velocity << 0.0, 0.0, Landing_velocity_*-1.0;
  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavVel_ << 0.0, 0.0, 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  load_control_=true;
  D_ << dx_, dy_, dz_;
  tau << tau_x, tau_y, tau_z;
  Cable_loop =ros::Time::now();
  creates();
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}
void geometricCtrl::batteryCallback(const sensor_msgs::BatteryState &msg){
  battery_voltage = msg.voltage;
  // ROS_INFO_STREAM("Battery "<<battery_voltage);
}
void geometricCtrl::gpsrawCallback(const sensor_msgs::NavSatFix &msg){
  if(!gps_home_init){
    gps_home_init = true;
     gps_home(0) = msg.latitude;
     gps_home(1) = msg.longitude;
     gps_home(2) = msg.altitude ; 
    LatLonToUTMXY(gps_home(0),gps_home(1),32,UTM_HOME_X,UTM_HOME_Y);//32 zurich 48 VietNam
  }
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),32,UTM_X,UTM_Y); //32 zurich 48 VietNam
 gps_pos(0) = UTM_X-UTM_HOME_X;
 gps_pos(1) = UTM_Y-UTM_HOME_Y;
//  ROS_INFO_STREAM("you are at "<<std::setprecision(20)<<"X:"<<UTM_X-UTM_HOME_X<<"  Y:"<<UTM_Y-UTM_HOME_Y);
}
void geometricCtrl::imuCallback(const sensor_msgs::Imu &msg){
Imu_base(0) = msg.linear_acceleration.x;
Imu_base(1) = msg.linear_acceleration.y;
Imu_base(2) = msg.linear_acceleration.z;
Imu_ang_vel(0)= msg.angular_velocity.x;
Imu_ang_vel(1)= msg.angular_velocity.y;
Imu_ang_vel(2)= msg.angular_velocity.z;
Eigen::Quaterniond quat_imu ;
quat_imu.w()=msg.orientation.w;
quat_imu.x()=msg.orientation.x;
quat_imu.y()=msg.orientation.y;
quat_imu.z()=msg.orientation.z;
Eigen::Vector3d Imu_accel = quat_imu * Imu_base ;  
// ROS_INFO_STREAM(Imu_accel+g_<<"imu_base");
}
void geometricCtrl::loadgroundtruth_callback(const nav_msgs::Odometry &msg){
  Load_pos_gth << msg.pose.pose.position.x , msg.pose.pose.position.y , msg.pose.pose.position.z ;
  Eigen::Vector3d temp;
  temp << -0.468285, -0.00529422, -0.258947;
  Load_pos_gth += temp ;
  temp << -0.274713, 0 , 0.0498823;
  Load_pos_gth -= temp ;
  Load_vel_gth << msg.twist.twist.linear.x , msg.twist.twist.linear.y , msg.twist.twist.linear.z ;
}
void geometricCtrl::imuphysicalCallback(const sensor_msgs::Imu &msg){
double t = ros::Time::now().toSec()-last_physical_load_time.toSec();
Imup_load_base(0) =msg.linear_acceleration.x;
Imup_load_base(1) =msg.linear_acceleration.y;
Imup_load_base(2) =msg.linear_acceleration.z;
Imup_load_quat.w()=msg.orientation.w;
Imup_load_quat.x()=msg.orientation.x;
Imup_load_quat.y()=msg.orientation.y;
Imup_load_quat.z()=msg.orientation.z;
Loadp_pos = mavPos_ + Imup_load_quat * Eigen::Vector3d::UnitZ()*-0.33;
Loadp_vel = (Loadp_pos - Loadp_last_pos )/t;
Loadp_last_pos = Load_pos;
last_physical_load_time = ros::Time::now();
Imup_load_ang_vel(0)= msg.angular_velocity.x;
Imup_load_ang_vel(1)= msg.angular_velocity.y;
Imup_load_ang_vel(2)= msg.angular_velocity.z;
Imup_load_ang_vel = Imup_load_quat * Imup_load_ang_vel;
Loadp_accel = Imup_load_quat * Imup_load_base ;
visualization_msgs::Marker marker;
marker.header.frame_id = "map";
marker.header.stamp = ros::Time();
marker.ns = "my_namespace";
marker.id = 0;
marker.type = visualization_msgs::Marker::ARROW;
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position.x = Loadp_pos(0);
marker.pose.position.y = Loadp_pos(1);
marker.pose.position.z = Loadp_pos(2);
marker.pose.orientation.x = msg.orientation.x;
marker.pose.orientation.y = msg.orientation.y;
marker.pose.orientation.z = msg.orientation.z;
marker.pose.orientation.w = msg.orientation.w;
marker.scale.x = 1;
marker.scale.y = 0.1;
marker.scale.z = 0.1;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 1.0;
marker.color.g = 1.0;
marker.color.b = 0.0;
ImuPmarker_pub.publish( marker );
}
void geometricCtrl::imuloadCallback(const sensor_msgs::Imu &msg){
double t = ros::Time::now().toSec()-last_load_time.toSec();
Imu_load_base(0) =msg.linear_acceleration.x;
Imu_load_base(1) =msg.linear_acceleration.y;
Imu_load_base(2) =msg.linear_acceleration.z;
Imu_load_quat.w()=msg.orientation.w;
Imu_load_quat.x()=msg.orientation.x;
Imu_load_quat.y()=msg.orientation.y;
Imu_load_quat.z()=msg.orientation.z;
Load_accel = Imu_load_quat * Imu_load_base + g_ ;
Eigen::Vector3d relative = Imu_load_quat * Eigen::Vector3d::UnitZ() * 0.4;
ROS_INFO_STREAM("line"<<relative(0)<<" "<<relative(1)<<" "<<relative(2));
Load_pos = mavPos_ - relative ;
Load_vel = (Load_pos - Load_last_pos )/t;
Load_last_pos = Load_pos;
last_load_time = ros::Time::now();
}
void geometricCtrl::globalCallback(const nav_msgs::Odometry &msg){
    globalPos_ = toEigen(msg.pose.pose.position);
    //ROS_INFO_STREAM("global_position"<< globalPos_(0)<<" "<< globalPos_(1)<<" "<< globalPos_(2));
    globalVel_ = toEigen(msg.twist.twist.linear);
    globalAtt_(0) = msg.pose.pose.orientation.w;
    globalAtt_(1) = msg.pose.pose.orientation.w;
    globalAtt_(2) = msg.pose.pose.orientation.w;
    globalAtt_(3) = msg.pose.pose.orientation.w;
}
void geometricCtrl::targetCallback(const geometry_msgs::TwistStamped &msg) {
  reference_request_last_ = reference_request_now_;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);

  if (reference_request_dt_ > 0)
    targetAcc_ = (targetVel_ - targetVel_prev_) / reference_request_dt_;
  else
    targetAcc_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);

  if (msg.type_mask == 1) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 2) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else if (msg.type_mask == 4) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();

  } else {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = toEigen(msg.jerk);
    targetSnap_ = toEigen(msg.snap);
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg) {
  if (!velocity_yaw_) mavYaw_ = double(msg.data);
}

void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  reference_request_last_ = reference_request_now_;

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  reference_request_now_ = ros::Time::now();
  reference_request_dt_ = (reference_request_now_ - reference_request_last_).toSec();

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();

  if (!velocity_yaw_) {
    Eigen::Quaterniond q(pt.transforms[0].rotation.w, pt.transforms[0].rotation.x, pt.transforms[0].rotation.y,
                         pt.transforms[0].rotation.z);
    Eigen::Vector3d rpy = Eigen::Matrix3d(q).eulerAngles(0, 1, 2);  // RPY
    mavYaw_ = rpy(2);
  }
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  mavPos_ = toEigen(msg.pose.position); //- Eigen::Vector3d::UnitX() * 0.4 ;
  mavAtt_.w() = msg.pose.orientation.w;
  mavAtt_.x() = msg.pose.orientation.x;
  mavAtt_.y() = msg.pose.orientation.y;
  mavAtt_.z() = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  last_landing_request = ros::Time::now();
  last_landing_command = ros::Time::now();
  if(request.data == true)
  landing_detected = true ;
  else landing_detected = false ;
  ROS_INFO_STREAM("landing _detected"<<landing_detected);
  node_state = LANDING;
  last_detected_pos = mavPos_;
  landing_pos = mavPos_;
  landing_vel = mavVel_;
  Landing_last_horizontal_error = horizontal_part(mavPos_)- Eigen::Vector3d::Zero();
  return true;
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
    case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose, "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      if(!load_control_)
      node_state = MISSION_EXECUTION;
      else
      node_state = LOAD_MISSION;
      break;
    case START:{
      if(mavVel_.norm()>0.1 && done_thrust_calib == false){
      done_thrust_calib = true;
      desired_acc << 0 , 0 ,-1;
      }
      if(done_thrust_calib == false){
      desired_acc << 0,0,5;
      thrust_start += 0.005;
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      cmdBodyRate_(3) = thrust_start;
      }
      else {
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      }
      pubRateCommands(cmdBodyRate_, q_des);
      ROS_INFO_STREAM("THRUST CALIBRATION"<<cmdBodyRate_(3));
      if(mavVel_.norm()<0.01 && mavPos_(3)<0.01 && done_thrust_calib == true){
      ROS_INFO_STREAM("Thrust calibrated"<< thrust_start);
      node_state = MISSION_EXECUTION;
      }
      break;
    }
    case MISSION_EXECUTION: {
      desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      pubRateCommands(cmdBodyRate_, q_des);

      break;
    }
     case LOAD_MISSION: {
      Eigen::Vector3d Load_des_acc = control_Load_Position(targetPos_, targetVel_, targetAcc_);
      computeLoadQuatCmd(Load_des_acc);    
      computeCableCmd(Load_des_acc);
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      pubRateCommands(cmdBodyRate_, q_des);
       double time = (ros::Time::now() -Flight_start).toSec();
      updates(time,targetPos_(0),targetPos_(1),targetPos_(2),Load_pos(0),Load_pos(1),Load_pos(2),mavPos_(0),mavPos_(1),mavPos_(2),Load_pos_gth(0),Load_pos_gth(1),Load_pos_gth(2),Load_vel_gth(0),Load_vel_gth(1),Load_vel_gth(2),Load_vel(0),Load_vel(1),Load_vel(2),0);
      break; 
}

    case LANDING: {
      
      Eigen::Vector3d landing_pos_horizontal = horizontal_part(landing_pos); 
      Eigen::Vector3d marker_horizontal =Eigen::Vector3d::Zero(); ;
      Eigen::Vector3d error_horizontal = marker_horizontal - landing_pos_horizontal ;
      Eigen::Vector3d error_axis;
      Eigen::Vector3d error_velocity ;

      // IF MARKER NOT DETECTED increases the height by 0.5 for evert 4 sec, max + 5m
      if(landing_detected == 0){ 
        int high_offset =  std::min((int)(ros::Time::now().toSec()-last_landing_request.toSec())/4,10);
        error_velocity = -Landing_velocity -landing_vel + last_detected_pos- landing_pos + Eigen::Vector3d(0,0, high_offset*0.5);
        error_axis = Eigen::Vector3d::Zero();
      }
      // IF MARKER DETECTED FLY HORIZONTALY to marker_axis , horizontal_vel = 0.3 m/s
      else if(error_horizontal.norm()> 0.2 || landing_vel.norm()> 0.2)
      {
        error_axis = Eigen::Vector3d::Zero();
        error_velocity = error_horizontal/error_horizontal.norm()*0.2 -landing_vel;
      }
      // IF DRONE REACH MARKER AXIS , START TO LAND (with marker continuing to update)
      else {
        error_axis = error_horizontal;
        error_velocity = Landing_velocity - landing_vel;
      }
      // SWITCH TO LANDED at set high
      if(landing_pos(2)<0.5) targetAcc_ += Eigen::Vector3d(0,0,-0.1)*landing_loop;
      if(mavVel_.norm()<0.1 && mavPos_(2)< 0.2) node_state = LANDED;
      targetAcc_(2) = std::max(-2.0,targetAcc_(2));

      // GEOMETRIC_CONTROLLER part (max accelerations = 2.0)
      Eigen::Vector3d error_landing = error_velocity * 6.0 + error_axis * 4.0 ; 
      if(error_landing.norm() > 2.0 ) error_landing = error_landing / error_landing.norm() *2.0 ;
      double landing_loop =  ros::Time::now().toSec()-last_landing_command.toSec();
      landing_vel += error_landing* landing_loop  ;
      landing_pos += landing_vel* landing_loop   ;
      Eigen::Vector3d Load_des_acc = control_Load_Position(targetPos_, targetVel_, targetAcc_);
      // computeCableCmd(Load_des_acc,);
      computeBodyRateCmd(cmdBodyRate_, desired_acc);
      pubRateCommands(cmdBodyRate_, q_des);
      last_landing_command = ros::Time::now();
      ros::spinOnce();
      break;
    }
    case LANDED:
      ROS_INFO("Landed. Please set to position control and disarm.");
      cmdloop_timer_.stop();
      break;
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { current_state_ = *msg; }

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
  if (sim_enable_) {
    // Enable OFFBoard mode and arm automatically
    // This will only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(5.0))) {
        if (arming_client_.call(arm_cmd_) && arm_cmd_.response.success) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}

void geometricCtrl::pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Quaterniond &target_attitude) {
  mavros_msgs::AttitudeTarget msg;

  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  msg.body_rate.x = cmd(0);
  msg.body_rate.y = cmd(1);
  msg.body_rate.z = cmd(2);
  msg.type_mask = 128;  // Ignore orientation messages
  msg.orientation.w = target_attitude.w();
  msg.orientation.x = target_attitude.x();
  msg.orientation.y = target_attitude.y();
  msg.orientation.z = target_attitude.z();
  msg.thrust = cmd(3);

  angularVelPub_.publish(msg);
}

void geometricCtrl::pubSystemStatus() {
  mavros_msgs::CompanionProcessStatus msg;

  msg.header.stamp = ros::Time::now();
  msg.component = 196;  // MAV_COMPONENT_ID_AVOIDANCE
  msg.state = (int)companion_state_;

  systemstatusPub_.publish(msg);
}

Eigen::Vector3d geometricCtrl::controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }
  const Eigen::Vector3d pos_error = mavPos_ - target_pos;// + load_pos_horizontal*0.1); 
  const Eigen::Vector3d vel_error = mavVel_ - target_vel ;
  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error);

  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = Eigen::Vector3d::Zero();

  // Reference acceleration
  Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;
  Eigen::Vector3d zb = mavAtt_ * Eigen::Vector3d::UnitZ();
  return a_des;
}
Eigen::Vector3d geometricCtrl::control_Load_Position(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                               const Eigen::Vector3d &target_acc) {
  /// Compute BodyRate commands using differential flatness
  /// Controller based on Faessler 2017
  const Eigen::Vector3d a_ref = target_acc;
  if (velocity_yaw_) {
    mavYaw_ = getVelocityYaw(mavVel_);
  }
  const Eigen::Vector3d pos_error = Load_pos - target_pos ;
  // ROS_INFO_STREAM ("Load_pos: " << Load_pos(0)<< " " << Load_pos(1)<< " " << Load_pos(2));
  // ROS_INFO_STREAM ("pos_error: " << pos_error(0)*-2<< " " << pos_error(1)*-2<< " " << pos_error(2)*-2);
  // ROS_INFO_STREAM ("Load_vel "<<Load_vel(0)<< " "<< Load_vel(1)<< " " << Load_vel(2));
  const Eigen::Vector3d vel_error = Load_vel - target_vel ;
  // ROS_INFO_STREAM ("vel_error: " << vel_error(0)*-8<< " " << vel_error(1)*-8<< " " << vel_error(2)*-10);
  // Position Controller
  const Eigen::Vector3d a_fb = pos_Load_controller(pos_error, vel_error);
  // ROS_INFO_STREAM ("a_fb "<<a_fb(0)<<" "<< a_fb(1)<<" "<< a_fb(2));
  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = Eigen::Vector3d::Zero();

  // Reference acceleration
  const Eigen::Vector3d a_des = a_fb + a_ref - a_rd - g_;
  return a_des;


}
void geometricCtrl::computeLoadQuatCmd( const Eigen::Vector3d &a_des) {
  // Reference attitude
  double loadYaw_ = 0.0;
  Eigen::Vector3d x_C = loadYaw_* Eigen::Vector3d::UnitX();
  Eigen::Vector3d y_C = loadYaw_ * Eigen::Vector3d::UnitY();
  Eigen::Vector3d z_B;
  if(a_des.norm()<0.01){
    z_B = Imu_load_quat * Eigen::Vector3d::UnitZ();
  }
  else z_B = a_des.normalized();
  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, mavAtt_);
  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  Eigen::Quaterniond desired_attitude(R_W_B);
  q_load_des = desired_attitude;
}
double geometricCtrl::ToEulerYaw(const Eigen::Quaterniond& q){
    Vector3f angles;    //yaw pitch roll
    const auto x = q.x();
    const auto y = q.y();
    const auto z = q.z();
    const auto w = q.w();
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
  }
void geometricCtrl::computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &a_des) {
  // Reference attitude
  Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(mavYaw_, Eigen::Vector3d::UnitZ()));
  Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
  Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
  Eigen::Vector3d z_B;
  if(a_des.norm()<0.01){
    z_B = mavAtt_ * Eigen::Vector3d::UnitZ();
  }
  else z_B = a_des.normalized();
  const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
  const Eigen::Vector3d x_B =
      computeRobustBodyXAxis(x_B_prototype, x_C, y_C, mavAtt_);
  const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
  const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
  Eigen::Quaterniond desired_attitude(R_W_B);
  q_des = desired_attitude;
  bodyrate_cmd = geometric_attcontroller(q_des, a_des, mavAtt_);  // Calculate BodyRate
}
void geometricCtrl::computeCableCmd(Eigen::Vector3d &a_des) {
  Eigen::Vector3d z_B;
  if(a_des.norm()<0.01){
    z_B = Imu_load_quat * Eigen::Vector3d::UnitZ();
  }
  else z_B = a_des.normalized();

  Eigen::Vector3d quad_pos_target =  targetPos_+ 0.4 * z_B;
  Eigen::Vector3d quad_vel_target =  targetVel_ ;
  desired_acc = controlPosition( quad_pos_target , quad_vel_target , Eigen::Vector3d::Zero());
  desired_acc +=  a_des/ 3 ;
} 

Eigen::Vector3d geometricCtrl::computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate){
  Eigen::Vector3d x_B = x_B_prototype;

  if (x_B.norm()<0.01) {
    // if cross(y_C, z_B) == 0, they are collinear =>
    // every x_B lies automatically in the x_C - z_C plane

    // Project estimated body x-axis into the x_C - z_C plane
    const Eigen::Vector3d x_B_estimated =
        attitude_estimate * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d x_B_projected =
        x_B_estimated - (x_B_estimated.dot(y_C)) * y_C;
    if (x_B_projected.norm()<0.01) {
      x_B = x_C;
    } else {
      x_B = x_B_projected.normalized();
    }
  } else {
    x_B.normalize();
  }
  return x_B;
}

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error;  // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}
Eigen::Vector3d geometricCtrl::pos_Load_controller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error) {
  Eigen::Vector3d a_fb =
      Kpos_load.asDiagonal() * pos_error + Kvel_load.asDiagonal() * vel_error;  // feedforward term for trajectory error

  if (a_fb.norm() > max_fb_acc_)
    a_fb = (max_fb_acc_ / a_fb.norm()) * a_fb;  // Clip acceleration if reference is too large

  return a_fb;
}

Eigen::Vector4d geometricCtrl::geometric_attcontroller(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc,
                                                       Eigen::Quaterniond &curr_att) {
  Eigen::Vector4d ratecmd;
  Eigen::Vector3d bodyrates;
  Eigen::Quaterniond q_e;
  q_e = curr_att.inverse() * ref_att ;
  if (q_e.w() <= 0) {
    bodyrates.x() = - krp * q_e.x();
    bodyrates.y() = - krp * q_e.y();
    bodyrates.z() = - kyaw * q_e.z();
  } else {
    bodyrates.x() = krp * q_e.x();
    bodyrates.y() = krp * q_e.y();
    bodyrates.z() = kyaw * q_e.z();
  }
  ratecmd.head(3) = bodyrates;
  Eigen::Vector3d zb = curr_att * Eigen::Vector3d::UnitZ();
  ratecmd(3) =
      std::max(0.0, std::min(1.0, norm_thrust_const_ * ref_acc.dot(zb) + norm_thrust_offset_));  // Calculate thrust
  return ratecmd;
}

bool geometricCtrl::ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
  unsigned char mode = req.data;

  ctrl_mode_ = mode;
  res.success = ctrl_mode_;
  res.message = "controller triggered";
  return true;
}

bool geometricCtrl::almostZero(double value) {
  if (fabs(value) < 0.01) return true;
  else return false;
}
void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,
                                               uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.2f ", config.max_acc);
  } else if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.2f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.2f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.2f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.2f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.2f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kv_z);
  }
  else if (Krp_ != config.Krp) {
    Krp_ = config.Krp;
    ROS_INFO("Reconfigure request : Krp  = %.2f  ", config.Krp);
  }
  else if (Kyaw_ != config.Kyaw) {
    Kyaw_ = config.Kyaw;
    ROS_INFO("Reconfigure request : Kv_z  = %.2f  ", config.Kyaw);
  }
  else if (norm_thrust_const_ != config.Thrust) {
    norm_thrust_const_ = config.Thrust;
    ROS_INFO("Reconfigure request : Thrust  = %.2f  ", config.Thrust);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  kyaw = Kyaw_;
  krp  = Krp_;
}
