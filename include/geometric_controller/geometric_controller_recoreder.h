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

#ifndef GEOMETRIC_CONTROLLER_H
#define GEOMETRIC_CONTROLLER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <tf/transform_broadcaster.h>

#include <stdio.h>
#include <cstdlib>
#include <sstream>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CompanionProcessStatus.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Float32.h>
#include <Eigen/Dense>

#include <visualization_msgs/Marker.h>
#include <controller_msgs/FlatTarget.h>
#include <dynamic_reconfigure/server.h>
#include <geometric_controller/GeometricControllerConfig.h>
#include <std_srvs/SetBool.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <trajectory_msgs/MultiDOFJointTrajectoryPoint.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>

#include "geometric_controller/common.h"
#include "UTM.h"
#include "logging_lib.cpp"
#include <cmath>

#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2
#define RPG_CONTROLLER 3

using namespace std;
using namespace Eigen;


class geometricCtrl {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber referenceSub_;
  ros::Subscriber flatreferenceSub_;
  ros::Subscriber multiDOFJointSub_;

  ros::Subscriber global_poseSub_;
  ros::Subscriber tf_marker;
  ros::Subscriber mavposeSub_, gzmavposeSub_;
  ros::Subscriber mavtwistSub_,batterySub_;

  ros::Subscriber yawreferenceSub_;
  ros::Time start;
  ros::Timer cmdloop_timer_;
  double marker_type = 0;
  bool velocity_yaw_;

  double last_yaw_ref = 0;


  double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
  Eigen::Vector3d mavPos_, mavVel_, mavRate_;
  Eigen::Vector3d last_ref_acc_{Eigen::Vector3d::Zero()};

  double mavYaw_;
  Eigen::Vector3d g_;
  Eigen::Quaterniond mavAtt_, q_des , q_load_des;


  Eigen::Vector3d globalPos_,gpsraw,gps_pos;
  Eigen::Vector3d h_Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d r_Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d p_Pos = Eigen::Vector3d::Zero();
  
  Eigen::Vector3d lh_Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d lr_Pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d lp_Pos = Eigen::Vector3d::Zero();


  float battery_voltage ;

  bool done_thrust_calib = false;

  void globalCallback(const nav_msgs::Odometry &msg);


  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void targetCallback(const geometry_msgs::TwistStamped &msg);
  void flattargetCallback(const controller_msgs::FlatTarget &msg);
  void yawtargetCallback(const std_msgs::Float32 &msg);
  void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
  void cmdloopCallback(const ros::TimerEvent &event);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void tf_markerCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void batteryCallback(const sensor_msgs::BatteryState &state);

  template <class T>
  void waitForPredicate(const T *pred, const std::string &msg, double hz = 2.0) {
    ros::Rate pause(hz);
    ROS_INFO_STREAM(msg);
    while (ros::ok() && !(*pred)) {
      ros::spinOnce();
      pause.sleep();
    }
  };
  geometry_msgs::Pose home_pose_;
  bool received_home_pose;
  bool gps_home_init = false;
  Eigen::Vector3d gps_home = Eigen::Vector3d::Zero();
 public:
  geometricCtrl(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~geometricCtrl();
 
};

#endif
