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
      nh_private_(nh_private)
       {
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());
  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavtwistSub_ = nh_.subscribe("mavros/local_position/velocity_local", 1, &geometricCtrl::mavtwistCallback, this,
                               ros::TransportHints().tcpNoDelay());

  tf_marker = nh_.subscribe("/tf_marker", 1, &geometricCtrl::tf_markerCallback, this,
                              ros::TransportHints().tcpNoDelay());
  cmdloop_timer_ = nh_.createTimer(ros::Duration(0.01), &geometricCtrl::cmdloopCallback,
                                   this);                           
  batterySub_= nh_.subscribe("mavros/battery", 1, &geometricCtrl::batteryCallback, this,
                              ros::TransportHints().tcpNoDelay());  
  start = ros::Time::now();
  creates();
  updates(0,0,0,0,0,00,0,0,0,0,0,00,0,0,0,0,00,0,0,0);
}
geometricCtrl::~geometricCtrl() {
  // Destructor
}
void geometricCtrl::batteryCallback(const sensor_msgs::BatteryState &msg){
  battery_voltage = msg.voltage;
  // ROS_INFO_STREAM("Battery "<<battery_voltage);
}


void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

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

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

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
  mavPos_ = toEigen(msg.pose.position); 
  mavAtt_.w() = msg.pose.orientation.w;
  mavAtt_.x() = msg.pose.orientation.x;
  mavAtt_.y() = msg.pose.orientation.y;
  mavAtt_.z() = msg.pose.orientation.z;
}

void geometricCtrl::tf_markerCallback(const geometry_msgs::PoseStamped &msg) {

  if (msg.header.frame_id[1]== 'r'){   
     r_Pos = toEigen(msg.pose.position);
     p_Pos = Eigen::Vector3d::Zero();
     h_Pos = Eigen::Vector3d::Zero();
     }
  if (msg.header.frame_id[1]== 'p'){
     p_Pos = toEigen(msg.pose.position);
     r_Pos = Eigen::Vector3d::Zero();
     h_Pos = Eigen::Vector3d::Zero();
  }
  if (msg.header.frame_id[1]== 'h'){ 
     h_Pos = toEigen(msg.pose.position);
     r_Pos  = Eigen::Vector3d::Zero();
     p_Pos = Eigen::Vector3d::Zero();
  }
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
}

void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
 double time = ros::Time::now().toSec() -start.toSec();

 if(!(r_Pos-lr_Pos).norm()> 0)  r_Pos = Eigen::Vector3d::Zero();
 if(!(p_Pos-lp_Pos).norm()> 0)  p_Pos = Eigen::Vector3d::Zero();
 if(!(h_Pos-lh_Pos).norm()> 0)  h_Pos = Eigen::Vector3d::Zero();
 
 if(r_Pos.norm() > 0) lr_Pos = r_Pos;
 if(p_Pos.norm() > 0) lp_Pos = p_Pos;
 if(h_Pos.norm() > 0) lh_Pos = h_Pos;

ROS_INFO_STREAM(h_Pos(0)<<" "<<lh_Pos(0));
 updates(time,targetPos_(0),targetPos_(1),targetPos_(2),mavPos_(0),mavPos_(1),mavPos_(2),mavVel_(0),mavVel_(1),mavVel_(2),h_Pos(0),h_Pos(1),h_Pos(2),r_Pos(0),r_Pos(1),r_Pos(2),p_Pos(0),p_Pos(1),p_Pos(2),0);
}



