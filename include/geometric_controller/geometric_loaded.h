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
#include <quadrotor_msgs/PositionCommand.h>
#include <visualization_msgs/Marker.h>
#include "FlatTarget.h"
#include "PositionCommand.h"
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
#include <deque>

#define ERROR_QUATERNION 1
#define ERROR_GEOMETRIC 2
#define RPG_CONTROLLER 3

using namespace std;
using namespace Eigen;

enum class MAV_STATE {
  MAV_STATE_UNINIT,
  MAV_STATE_BOOT,
  MAV_STATE_CALIBRATIN,
  MAV_STATE_STANDBY,
  MAV_STATE_ACTIVE,
  MAV_STATE_CRITICAL,
  MAV_STATE_EMERGENCY,
  MAV_STATE_POWEROFF,
  MAV_STATE_FLIGHT_TERMINATION,
};

struct marker {
Eigen::Vector3d relative_pose;
Eigen::Vector3d local_pose;
double trust_coeff;
ros::Time stamp;
};

class geometricLoaded {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber referenceSub_;
  ros::Subscriber flatreferenceSub_,quad_msgsSub_,rotorTmSub_;
  ros::Subscriber multiDOFJointSub_;
  ros::Subscriber mavstateSub_;
  ros::Subscriber gpsrawSub_;
  ros::Subscriber global_poseSub_;
  ros::Subscriber marker_relative_Sub_;
  ros::Subscriber mavposeSub_, gzmavposeSub_;
  ros::Subscriber mavtwistSub_,batterySub_;
  ros::Subscriber imuSub_,imuloadSub_,imu_physicalSub_,load_ground_truthSub_;
  ros::Subscriber yawreferenceSub_;
  ros::Publisher rotorVelPub_, angularVelPub_, target_pose_pub_;
  ros::Publisher referencePosePub_;
  ros::Publisher posehistoryPub_,loadhistoryPub_;
  ros::Publisher systemstatusPub_;
  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;
  ros::ServiceServer ctrltriggerServ_;
  ros::ServiceServer land_service_;
  ros::Timer cmdloop_timer_, statusloop_timer_;
  ros::Time Cable_loop,Flight_start,last_request_, reference_request_now_, reference_request_last_ , last_landing_request ,last_load_time,last_physical_load_time, last_landing_command ,last_marker_control;
  string mav_name_;
  bool fail_detec_, ctrl_enable_, feedthrough_enable_,load_control_;
  int ctrl_mode_;
  int cross_status,cross_counter;
  double cross_sum,cross_average,cross_last; 
  bool landing_commanded_,landing_detected = false,Basending_thrust = true ;
  bool sim_enable_;
  int landing = 0;
  double quad_mass , load_mass ,cable_length ;
  int gps_enable = 1 ;
  bool velocity_yaw_;
  double kp_rot_, kd_rot_;
  double reference_request_dt_;
  double UTM_X,UTM_Y,UTM_HOME_X,UTM_HOME_Y;
  double attctrl_tau_;
  double norm_thrust_const_, norm_thrust_offset_;
  double max_fb_acc_;
  double dx_, dy_, dz_;
  double last_yaw_ref = 0;
  double drone_max_accel =25.0;
  double k_thrust_horz = 0 ;
  double drone_max_ver_acc =18.0;
  double start_norm_thrust = 0.2;
  double landing_loop = 1.0,last_yaw_X;
  deque<marker> dq_markers;
  marker last_marker;
  mavros_msgs::State current_state_;
  mavros_msgs::SetMode offb_set_mode_;
  mavros_msgs::CommandBool arm_cmd_;
  std::vector<geometry_msgs::PoseStamped> posehistory_vector_,loadhistory_vector_;
  MAV_STATE companion_state_ = MAV_STATE::MAV_STATE_ACTIVE;
  
  double initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;
  Eigen::Vector3d targetPos_, targetVel_, targetAcc_, targetJerk_, targetSnap_, targetPos_prev_, targetVel_prev_;
  Eigen::Vector3d mavPos_, mavVel_, mavRate_;
  Eigen::Vector3d last_ref_acc_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d landing_vel,landing_pos;
  Eigen::Vector3d integral_marker;
  double mavYaw_;
  Eigen::Vector3d g_;
  Eigen::Vector3d Start_pos,Start_kpos,Start_kvel;
  Eigen::Quaterniond mavAtt_, q_des , q_load_des;
  Eigen::Vector4d cmdBodyRate_;  //{wx, wy, wz, Thrust}
  Eigen::Vector3d Kpos_, Kvel_, D_,Kpos_load,Kvel_load;
  Eigen::Vector3d a0, a1, tau;
  Eigen::Vector3d globalPos_,gpsraw,gps_pos;
  Eigen::Vector3d globalVel_;
  Eigen::Vector3d Imu_base,Imu_load_base,Imup_load_base,Imu_load_ang_vel,Imup_load_ang_vel;
  Eigen::Quaterniond Imu_load_quat,Imup_load_quat;
  Eigen::Vector3d Load_accel,Loadp_accel,Load_pos_gth,Load_vel_gth;
  Eigen::Vector3d Imu_accel;
  Eigen::Vector3d desired_acc,xi_dot;
  Eigen::Vector3d Imu_ang_vel;
  Eigen::Vector3d drag_accel,integral_error;
  Eigen::Vector4d globalAtt_;
  Eigen::Vector3d Load_pos,Loadp_pos,Load_vel,Loadp_vel,Last_quad_target_pos = Eigen::Vector3d::Zero();
  Eigen::Vector3d Landing_velocity,Landing_last_horizontal_error;
  double Landing_velocity_;
  double tau_x, tau_y, tau_z;
  float battery_voltage ;
  double krp = 2.0, kyaw = 2.0, kvel_landing = 7,kpos_landing = 3,krp_start = 4.0 ;
  double Kpos_x_, Kpos_y_, Kpos_z_, Kvel_x_, Kvel_y_, Kvel_z_ , Krp_ , Kyaw_ , Kpos_load_xy ,Kpos_load_z ,Kvel_load_xy ,Kvel_load_z;
  int posehistory_window_;
  bool done_thrust_calib = false;
  void imuCallback(const sensor_msgs::Imu &msg);;
  void imuloadCallback(const sensor_msgs::Imu &msg);
  void imuphysicalCallback(const sensor_msgs::Imu &msg);
  void pubMotorCommands();
  void globalCallback(const nav_msgs::Odometry &msg);
  void gpsrawCallback(const sensor_msgs::NavSatFix &msg);
  void pubRateCommands(const Eigen::Vector4d &cmd, const Eigen::Quaterniond &target_attitude);
  void pubReferencePose(const Eigen::Vector3d &target_position, const Eigen::Vector4d &target_attitude);
  void pubPoseHistory();
  void pubSystemStatus();
  void odomCallback(const nav_msgs::OdometryConstPtr &odomMsg);
  void targetCallback(const geometry_msgs::TwistStamped &msg);
  void flattargetCallback(const controller_msgs::FlatTarget &msg);
  void quad_msgsCallback(const quadrotor_msgs::PositionCommand &msg);
  void rotorTmCallback(const controller_msgs::PositionCommand &msg);
  void yawtargetCallback(const std_msgs::Float32 &msg);
  void landing_state_trigger(int state);
  void multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
  void loadgroundtruth_callback(const nav_msgs::Odometry &msg);
  void keyboardCallback(const geometry_msgs::Twist &msg);
  void cmdloopCallback(const ros::TimerEvent &event);
  void mavstateCallback(const mavros_msgs::State::ConstPtr &msg);
  void mavposeCallback(const geometry_msgs::PoseStamped &msg);
  void mavtwistCallback(const geometry_msgs::TwistStamped &msg);
  void statusloopCallback(const ros::TimerEvent &event);
  void markerCallback(const geometry_msgs::PoseStamped &msg);
  void insert_marker_deque();
  void delete_marker_deque();
  void batteryCallback(const sensor_msgs::BatteryState &state);
  bool ctrltriggerCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
  bool landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response);
  bool almostZero(double value);
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d position,Eigen::Quaterniond quaternion) ;
  void loadStartparams();
  void loadFlyparams();
  void ascending_thrust();
  void appendPoseHistory(); 
  int check_cross();
  geometry_msgs::PoseStamped vector3d2PoseStampedMsg(Eigen::Vector3d &position, Eigen::Vector4d &orientation);
  void computeBodyRateCmd(Eigen::Vector4d &bodyrate_cmd, const Eigen::Vector3d &target_acc);
  void computeLoadQuatCmd(const Eigen::Vector3d &target_acc );
  void computeCableCmd(Eigen::Vector3d &load_target_acc , Eigen::Quaterniond &quat);
  double ToEulerYaw(const Eigen::Quaterniond& q); 
  Eigen::Vector3d controlPosition(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                  const Eigen::Vector3d &target_acc);
  Eigen::Vector3d controlMarker();
  Eigen::Vector3d control_Load_Position(const Eigen::Vector3d &target_pos, const Eigen::Vector3d &target_vel,
                                  const Eigen::Vector3d &target_acc);
  Eigen::Vector3d poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error);
  Eigen::Vector3d pos_Load_controller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error, const Eigen::Vector3d &itg_error);
  Eigen::Vector4d attcontroller(const Eigen::Vector4d &ref_att, const Eigen::Vector3d &ref_acc,
                                Eigen::Vector4d &curr_att);
  Eigen::Vector4d geometric_attcontroller(const Eigen::Quaterniond &ref_att, const Eigen::Vector3d &ref_acc,
                                          Eigen::Quaterniond &curr_att);
  Eigen::Vector4d jerkcontroller(const Eigen::Vector3d &ref_jerk, const Eigen::Vector3d &ref_acc,
                                 Eigen::Vector4d &ref_att, Eigen::Vector4d &curr_att);
  Eigen::Vector3d computeRobustBodyXAxis(const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate);
  enum FlightState { WAITING_FOR_HOME_POSE, MISSION_EXECUTION, LANDING, LANDED, START,LOAD_MISSION} node_state;

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
  void dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config, uint32_t level);
  geometricLoaded(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  virtual ~geometricLoaded();
  void getErrors(Eigen::Vector3d &pos, Eigen::Vector3d &vel) {
    pos = mavPos_ - targetPos_;
    vel = mavVel_ - targetVel_;
  };
  Eigen::Vector3d horizontal_part(Eigen::Vector3d a){
    Eigen::Vector3d new_a ; new_a << a(0), a(1) ,0.0;
    return new_a;
  }
  void setBodyRateCommand(Eigen::Vector4d bodyrate_command) { cmdBodyRate_ = bodyrate_command; };
  void setFeedthrough(bool feed_through) { feedthrough_enable_ = feed_through; };
  void setDesiredAcceleration(Eigen::Vector3d &acceleration) { targetAcc_ = acceleration; };
  static Eigen::Vector4d acc2quaternion(const Eigen::Vector3d &vector_acc, const double &yaw);
  static double getVelocityYaw(const Eigen::Vector3d velocity) { return atan2(velocity(1), velocity(0)); };
};

#endif
