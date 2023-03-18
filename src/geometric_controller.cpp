
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
  
  flatreferenceSub_ = nh_.subscribe("reference/flatsetpoint", 1, &geometricCtrl::flattargetCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  quad_msgsSub_ = nh_.subscribe("/planning/pos_cmd", 1, &geometricCtrl::quad_msgsCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  yawreferenceSub_ =
      nh_.subscribe("reference/yaw", 1, &geometricCtrl::yawtargetCallback, this, ros::TransportHints().tcpNoDelay());

  multiDOFJointSub_ = nh_.subscribe("command/trajectory", 1, &geometricCtrl::multiDOFJointCallback, this,
                                    ros::TransportHints().tcpNoDelay());
  mavstateSub_ =
      nh_.subscribe("mavros/state", 1, &geometricCtrl::mavstateCallback, this, ros::TransportHints().tcpNoDelay());
  mavposeSub_ = nh_.subscribe("mavros/local_position/pose", 1, &geometricCtrl::mavposeCallback, this,
                              ros::TransportHints().tcpNoDelay());
  mavrcSubs_ = nh_.subscribe("mavros/rc/in", 1, &geometricCtrl::rcCallback, this,
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
  rvisualize_pub= nh_.advertise<visualization_msgs::Marker>( "drone/rvisualize", 0 );
  angularVelPub_ = nh_.advertise<mavros_msgs::AttitudeTarget>("command/bodyrate_command", 1);
  referencePosePub_ = nh_.advertise<geometry_msgs::PoseStamped>("reference/pose", 1);
  local_gps_pub = nh_.advertise<geometry_msgs::PoseStamped>("debug/localgps_pos", 1);
  accel_command_pub = nh_.advertise<geometry_msgs::Point>("debug/accel_command", 1);
  setpoint_raw_Pub = nh_.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 1);
  reference_pose_pub_ = nh_.advertise<geometry_msgs::Quaternion>("debug/pose_command", 1);
  target_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
  posehistoryPub_ = nh_.advertise<nav_msgs::Path>("geometric_controller/path", 10);
  systemstatusPub_ = nh_.advertise<mavros_msgs::CompanionProcessStatus>("mavros/companion_process/status", 1);
  arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
  set_mode_client_ = nh_.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
  land_service_ = nh_.advertiseService("land", &geometricCtrl::landCallback, this);
  imuSub_ = nh_.subscribe("/mavros/imu/data",1,&geometricCtrl::imuCallback, this,
                              ros::TransportHints().tcpNoDelay());                                    
  nh_private_.param<string>("mavname", mav_name_, "iris");
  nh_private_.param<int>("ctrl_mode", ctrl_mode_, 2);
  nh_private_.param<bool>("enable_sim", sim_enable_, true);
  nh_private_.param<bool>("load_control_", load_control_, false);
  nh_private_.param<bool>("velocity_yaw", velocity_yaw_, false);
  nh_private_.param<double>("max_acc", max_fb_acc_, 9.0);
  nh_private_.param<double>("yaw_heading", mavYaw_, 0.0);
  nh_private_.param<double>("drag_dx", dx_, 0.0);
  nh_private_.param<double>("drag_dy", dy_, 0.0);
  nh_private_.param<double>("drag_dz", dz_, 0.0);
  nh_private_.param<double>("attctrl_constant", attctrl_tau_, 0.1);
  nh_private_.param<double>("normalizedthrust_constant", norm_thrust_const_, 0.05);  // 1 / max acceleration
  nh_private_.param<double>("normalizedthrust_offset", norm_thrust_integral_, 0.0);    // 1 / max acceleration
  nh_private_.param<double>("Kp_x", Kpos_x_, 8.0);
  nh_private_.param<double>("Kp_y", Kpos_y_, 8.0);
  nh_private_.param<double>("Kp_z", Kpos_z_, 10.0);
  nh_private_.param<double>("Kv_x", Kvel_x_, 1.5);
  nh_private_.param<double>("Kv_y", Kvel_y_, 1.5);
  nh_private_.param<double>("Kv_z", Kvel_z_, 3.3);
  nh_private_.param<double>("Ki_x", Kint_x_, 0.1);
  nh_private_.param<double>("Ki_y", Kint_y_, 0.1);
  nh_private_.param<double>("Ki_z", Kint_z_, 0.1);
  nh_private_.param<double>("Krp",  Krp_, 1.5);
  nh_private_.param<double>("Kyaw", Kyaw_, 2);
  nh_private_.param<double>("Ki_t", Kint_t_, 0.008);
  nh_private_.param<double>("Kd_t", Kder_t_, 0.16);
  nh_private_.param<int>("posehistory_window", posehistory_window_, 200);
  nh_private_.param<int>("test", test_, 0); 
  nh_private_.param<double>("init_pos_x", initTargetPos_x_, 0.0);
  nh_private_.param<double>("init_pos_y", initTargetPos_y_, 0.0);
  nh_private_.param<double>("init_pos_z", initTargetPos_z_, 5.0);
  nh_private_.param<double>("land_vel", Landing_velocity_, 1);
 
  Landing_velocity << 0.0, 0.0, Landing_velocity_*-1.0;
  targetPos_ << initTargetPos_x_, initTargetPos_y_, initTargetPos_z_;  // Initial Position
  control_mask == ACCELERATION_FEEDBACK;
  targetVel_ << 0.0, 0.0, 0.0;
  mavPos_ << 0.0, 0.0, 0.0;
  mavYaw_ = 0.0 ;
  mavVel_ << 0.0, 0.0, 0.0;
  g_ << 0.0, 0.0, -9.8;
  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  Kint_ << -Kint_x_, -Kint_y_, -Kint_z_;
  load_control_= false;
  rcHold = false;
  D_ << dx_, dy_, dz_;
  tau << tau_x, tau_y, tau_z;
  last_integral_error = Eigen::Vector3d::Zero();
  creates();
  Flight_start = ros::Time::now();
  oscillation_high_peak.push_back(0);
  oscillation_high_peak_time.push_back(0);
  oscillation_low_peak.push_back(0);
  oscillation_low_peak_time.push_back(0);
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
    LatLonToUTMXY(gps_home(0),gps_home(1),48,UTM_HOME_X,UTM_HOME_Y);//32 zurich 48 VietNam
  }
 gpsraw(0) = msg.latitude;
 gpsraw(1) = msg.longitude;
 gpsraw(2) = msg.altitude ;
 LatLonToUTMXY(gpsraw(0),gpsraw(1),48,UTM_X,UTM_Y); //32 zurich 48 VietNam
 gps_pos(0) = UTM_X-UTM_HOME_X;
 gps_pos(1) = UTM_Y-UTM_HOME_Y;
 gps_pos(2) = mavPos_(2);
 geometry_msgs::PoseStamped tx = toGeometry_msgs(gps_pos,mavAtt_);
 local_gps_pub.publish(tx);
}
void geometricCtrl::rcCallback(const mavros_msgs::RCIn &msg){
  if ( msg.channels.at(7) > 1500 )
  rcHold = true ;
  else 
  rcHold = false;
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
Imu_accel = quat_imu * Imu_base ;  
// ROS_INFO_STREAM(Imu_accel+g_<<"imu_base");
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
  control_mask == ACCELERATION_FEEDBACK;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;
  targetPos_ = toEigen(msg.twist.angular);
  targetVel_ = toEigen(msg.twist.linear);
  targetAcc_ = Eigen::Vector3d::Zero();
}
void geometricCtrl::quad_msgsCallback(const controller_msgs::PositionCommand &msg) {
  control_mask == ACCELERATION_FEEDBFORWARD;
  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);
  targetAcc_ = toEigen(msg.acceleration);
  mavYaw_ = double(msg.yaw);
  mavVelYaw_ = double(msg.yaw_dot);
}

void geometricCtrl::flattargetCallback(const controller_msgs::FlatTarget &msg) {

  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;
  control_mask =  Controller_mask(msg.type_mask);

  targetPos_ = toEigen(msg.position);
  targetVel_ = toEigen(msg.velocity);
  if (control_mask == ACCELERATION_FEEDBACK) {
    targetAcc_ = Eigen::Vector3d::Zero();
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    targetStarted = true;

  } else if (control_mask == ACCELERATION_FEEDBFORWARD) {
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    targetStarted = true;

  }else if (control_mask == VELOCITY_CONTROL) {
    targetPos_ = Eigen::Vector3d::Zero();
    targetAcc_ = toEigen(msg.acceleration);
    targetJerk_ = Eigen::Vector3d::Zero();
    targetSnap_ = Eigen::Vector3d::Zero();
    targetStarted = true;
  }
}

void geometricCtrl::yawtargetCallback(const std_msgs::Float32 &msg) {
  if (!velocity_yaw_) mavYaw_ = double(msg.data);
}


void geometricCtrl::multiDOFJointCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  trajectory_msgs::MultiDOFJointTrajectoryPoint pt = msg.points[0];
  control_mask == ACCELERATION_FEEDBFORWARD;
  targetPos_prev_ = targetPos_;
  targetVel_prev_ = targetVel_;

  targetPos_ << pt.transforms[0].translation.x, pt.transforms[0].translation.y, pt.transforms[0].translation.z;
  targetVel_ << pt.velocities[0].linear.x, pt.velocities[0].linear.y, pt.velocities[0].linear.z;

  targetAcc_ << pt.accelerations[0].linear.x, pt.accelerations[0].linear.y, pt.accelerations[0].linear.z;
  targetJerk_ = Eigen::Vector3d::Zero();
  targetSnap_ = Eigen::Vector3d::Zero();
}

void geometricCtrl::mavposeCallback(const geometry_msgs::PoseStamped &msg) {
  if (!received_home_pose) {
    received_home_pose = true;
    home_pose_ = msg.pose;
    ROS_INFO_STREAM("Home pose initialized to: " << home_pose_);
  }
  if( test_!= 1)
  mavPos_ = toEigen(msg.pose.position); //- Eigen::Vector3d::UnitX() * 0.4 ;
  mavAtt_.w() = msg.pose.orientation.w;
  mavAtt_.x() = msg.pose.orientation.x;
  mavAtt_.y() = msg.pose.orientation.y;
  mavAtt_.z() = msg.pose.orientation.z;
}

void geometricCtrl::mavtwistCallback(const geometry_msgs::TwistStamped &msg) {
  if(test_!=1){
  mavVel_ = toEigen(msg.twist.linear);
  mavRate_ = toEigen(msg.twist.angular);
  }
}

bool geometricCtrl::landCallback(std_srvs::SetBool::Request &request, std_srvs::SetBool::Response &response) {
  landing_commanded_ = !landing_commanded_;
  rcHold = !rcHold;
  return true;
}

void geometricCtrl::rawsetpointPub(const bool started, const Eigen::Vector3d &accelration,double yawvel){
  if(started){
      mavros_msgs::PositionTarget a;
      a.type_mask = (a.IGNORE_PX|a.IGNORE_PY|a.IGNORE_PZ|a.IGNORE_VX|a.IGNORE_VY|a.IGNORE_VZ|a.IGNORE_YAW);
      a.acceleration_or_force = toVector3(desired_acc);
      a.yaw_rate = yawvel;
      a.coordinate_frame = a.FRAME_LOCAL_NED;
      setpoint_raw_Pub.publish(a);
  }
  else {
    mavros_msgs::PositionTarget a;
      a.type_mask = (a.IGNORE_AFX|a.IGNORE_AFY|a.IGNORE_AFZ|a.IGNORE_VX|a.IGNORE_VY|a.IGNORE_VZ|a.IGNORE_YAW);
      a.position = toGeometry_msgs(targetPos_);
      a.yaw_rate = 0.0f;
      a.coordinate_frame = a.FRAME_LOCAL_NED;
      setpoint_raw_Pub.publish(a);
  }
}
void geometricCtrl::cmdloopCallback(const ros::TimerEvent &event) {
  switch (node_state) {
    case WAITING_FOR_HOME_POSE:
      waitForPredicate(&received_home_pose , "Waiting for home pose...");
      ROS_INFO("Got pose! Drone Ready to be armed.");
      node_state = MISSION_EXECUTION;
      break;
    case MISSION_EXECUTION: {
      desired_acc = controlPosition(targetPos_, targetVel_, targetAcc_);
      yaw_velocity = controlyawvel();
      pubDebugInfo(desired_acc,targetPos_,targetVel_);
      rawsetpointPub(targetStarted,desired_acc,yaw_velocity);
      rvisualize();
      updates((ros::Time::now()-Flight_start).toSec(),mavPos_(0),mavPos_(1),mavPos_(2),norm_thrust_const_,desired_acc(0),desired_acc(1),desired_acc(2),integral_error(0),integral_error(1),integral_error(2),Imu_accel(0),Imu_accel(1),Imu_accel(2),mavVel_(0),mavVel_(1),mavVel_(2), 0,0,0);
      // costfunction(mavPos_((int)tunning_),targetPos_((int)tunning_),desired_acc((int)tunning_), tunning_cost , (ros::Time::now()-Tunning_start).toSec());
      if (rcHold == true ) node_state = TUNNING;
      break;
    }
    case TUNNING:{
      break;
    }
    case LANDED:
      ROS_INFO("Landed. Please set to position control and disarm.");
      cmdloop_timer_.stop();
      break;
  }
}

void geometricCtrl::mavstateCallback(const mavros_msgs::State::ConstPtr &msg) { 
  current_state_ = *msg;
  }

void geometricCtrl::statusloopCallback(const ros::TimerEvent &event) {
  if (sim_enable_) {
    // Enable OFFBoard mode and arm automatically
    // This will only run if the vehicle is simulated
    arm_cmd_.request.value = true;
    offb_set_mode_.request.custom_mode = "OFFBOARD";
    if (current_state_.mode != "OFFBOARD" && (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
      if (set_mode_client_.call(offb_set_mode_) && offb_set_mode_.response.mode_sent) {
        ROS_INFO("Offboard enabled");
      }
      last_request_ = ros::Time::now();
    } else {
      if (!current_state_.armed && (ros::Time::now() - last_request_ > ros::Duration(2.0))) {
        arming_client_.call(arm_cmd_);
        if (arm_cmd_.response.success == true) {
          ROS_INFO("Vehicle armed");
        }
        last_request_ = ros::Time::now();
      }
    }
  }
  pubSystemStatus();
}
void geometricCtrl::pubDebugInfo(const Eigen::Vector3d &acc_, const Eigen::Vector3d &pos_ , const Eigen::Vector3d &vel_){
geometry_msgs::Point msg = toGeometry_msgs(acc_);
accel_command_pub.publish(msg);
msg = toGeometry_msgs(acc_);
accel_command_pub.publish(msg);
msg = toGeometry_msgs(acc_);
// reference_pose_pub_.publish(msg2);

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
  const Eigen::Vector3d pos_error = mavPos_ - target_pos;// + load_pos_horizontal*0.1); 
  const Eigen::Vector3d vel_error = mavVel_ - target_vel ;
  // Position Controller
  const Eigen::Vector3d a_fb = poscontroller(pos_error, vel_error,control_mask);
  // Rotor Drag compensation
  const Eigen::Vector3d a_rd = Eigen::Vector3d::Zero();
  // Reference acceleration
  Eigen::Vector3d a_des = a_fb + a_ref - a_rd ;//- g_;
  Eigen::Vector3d zb = mavAtt_ * Eigen::Vector3d::UnitZ();
  return a_des;
}
double geometricCtrl::controlyawvel(){
  double yawvel = mavYaw_ - ToEulerYaw(mavAtt_);
      if (fabs(yawvel)> fabs(yawvel-2*3.14159)) yawvel = (yawvel-2*3.14159)*0.5;
      else if (fabs(yawvel)> fabs(yawvel+2*3.14159)) yawvel = (yawvel+2*3.14159)*0.5;
      else yawvel = 0.5 * yawvel;
   return yawvel + 0.5 * mavVelYaw_;
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

Eigen::Vector3d geometricCtrl::poscontroller(const Eigen::Vector3d &pos_error, const Eigen::Vector3d &vel_error , int control_mask_) {
  Eigen::Vector3d a_fb;
  if(control_mask_ == ACCELERATION_FEEDBACK || control_mask_ == ACCELERATION_FEEDBFORWARD ){
  a_fb = Kpos_.asDiagonal() * pos_error + Kvel_.asDiagonal() * vel_error ;
  last_integral_error = integral_error;
  if(current_state_.mode == "OFFBOARD" && current_state_.armed)
  integral_handle(integral_error,a_fb,-0.01,Kint_,1.5);
  a_fb += integral_error;   
  } 
  else if(control_mask_ == VELOCITY_CONTROL ){ 
  Eigen::Vector3d One ;
  One << 1.0 ,1.0 ,1.0 ;
  if(current_state_.mode == "OFFBOARD" && current_state_.armed)
  integral_handle(integral_error,vel_error,0.01,-0.02*One,1.0); 
  a_fb = Kvel_.asDiagonal() * vel_error + integral_error;  // feedforward term for trajectory error
  }
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
  Throttle = ref_acc.dot(zb);
  if(current_state_.mode == "OFFBOARD" && current_state_.armed) 
  {
    integral_handle(norm_thrust_integral_,integral_error.dot(zb),0.01,Kint_t_,3.0,0.06);
    norm_thrust_derivative_ = (integral_error.dot(zb) - last_integral_error.dot(zb)) / 0.01 * Kder_t_;
  }
  ratecmd(3) =
      std::max(0.0, std::min(1.0, (norm_thrust_const_ + norm_thrust_integral_ + norm_thrust_derivative_) * ref_acc.dot(zb)));  // Calculate thrust
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

void geometricCtrl::rvisualize(){
visualization_msgs::Marker marker;
marker.header.frame_id = "map";
marker.header.stamp = ros::Time();
marker.ns = "drone";
marker.id = 0;
marker.type = visualization_msgs::Marker::MESH_RESOURCE;
marker.mesh_resource = "package://geometric_controller/mesh/quadrotor_2.stl";
marker.action = visualization_msgs::Marker::ADD;
marker.pose.position = toGeometry_msgs(mavPos_);
marker.pose.orientation = toGeometry_msgs(mavAtt_);
marker.scale.x = 1.5;
marker.scale.y = 1.5;
marker.scale.z = 3.0;
marker.color.a = 1.0; // Don't forget to set the alpha!
marker.color.r = 0.01;
marker.color.g = 0.01;
marker.color.b = 0.01;
rvisualize_pub.publish( marker);
}
void geometricCtrl::dynamicReconfigureCallback(geometric_controller::GeometricControllerConfig &config,
                                               uint32_t level) {
  if (max_fb_acc_ != config.max_acc) {
    max_fb_acc_ = config.max_acc;
    ROS_INFO("Reconfigure request : max_acc = %.4f ", config.max_acc);
  } else if (Kpos_x_ != config.Kp_x) {
    Kpos_x_ = config.Kp_x;
    ROS_INFO("Reconfigure request : Kp_x  = %.4f  ", config.Kp_x);
  } else if (Kpos_y_ != config.Kp_y) {
    Kpos_y_ = config.Kp_y;
    ROS_INFO("Reconfigure request : Kp_y  = %.4f  ", config.Kp_y);
  } else if (Kpos_z_ != config.Kp_z) {
    Kpos_z_ = config.Kp_z;
    ROS_INFO("Reconfigure request : Kp_z  = %.4f  ", config.Kp_z);
  } else if (Kvel_x_ != config.Kv_x) {
    Kvel_x_ = config.Kv_x;
    ROS_INFO("Reconfigure request : Kv_x  = %.4f  ", config.Kv_x);
  } else if (Kvel_y_ != config.Kv_y) {
    Kvel_y_ = config.Kv_y;
    ROS_INFO("Reconfigure request : Kv_y =%.4f  ", config.Kv_y);
  } else if (Kvel_z_ != config.Kv_z) {
    Kvel_z_ = config.Kv_z;
    ROS_INFO("Reconfigure request : Kv_z  = %.4f  ", config.Kv_z);
  } else if (Kint_x_ != config.Ki_x) {
    Kint_x_ = config.Ki_x;
    ROS_INFO("Reconfigure request : Ki_x  = %.4f  ", config.Ki_x);
  } else if (Kint_y_ != config.Ki_y) {
    Kint_y_ = config.Ki_y;
    ROS_INFO("Reconfigure request : Ki_y =%.4f  ", config.Ki_y);
  } else if (Kint_z_ != config.Ki_z) {
    Kint_z_ = config.Ki_z;
    ROS_INFO("Reconfigure request : Ki_z  = %.4f  ", config.Ki_z);
  }
  else if (Krp_ != config.Krp) {
    Krp_ = config.Krp;
    ROS_INFO("Reconfigure request : Krp  = %.4f  ", config.Krp);
  }
  else if (Kyaw_ != config.Kyaw) {
    Kyaw_ = config.Kyaw;
    ROS_INFO("Reconfigure request : Kyaw  = %.4f  ", config.Kyaw);
  }
  else if (norm_thrust_const_ != config.Thrust) {
    norm_thrust_const_ = config.Thrust;
    ROS_INFO("Reconfigure request : Thrust  = %.4f  ", config.Thrust);
  }

  Kpos_ << -Kpos_x_, -Kpos_y_, -Kpos_z_;
  Kvel_ << -Kvel_x_, -Kvel_y_, -Kvel_z_;
  Kint_ << -Kint_x_, -Kint_y_, -Kint_z_;
  kyaw = Kyaw_;
  krp  = Krp_;
}

