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
 * @brief Common library
 *
 * Common library for geometric controller
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#ifndef COMMON_H
#define COMMON_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <Eigen/Dense>

static Eigen::Matrix3d matrix_hat(const Eigen::Vector3d &v) {
  Eigen::Matrix3d m;
  // Sanity checks on M
  m << 0.0, -v(2), v(1), v(2), 0.0, -v(0), -v(1), v(0), 0.0;
  return m;
}

static Eigen::Vector3d matrix_hat_inv(const Eigen::Matrix3d &m) {
  Eigen::Vector3d v;
  // TODO: Sanity checks if m is skew symmetric
  v << m(7), m(2), m(3);
  return v;
}

Eigen::Vector3d toEigen(const geometry_msgs::Point &p) {
  Eigen::Vector3d ev3(p.x, p.y, p.z);
  return ev3;
}

Eigen::Vector3d toEigen(const geometry_msgs::PoseStamped &p) {
  return toEigen(p.pose.position);
}
Eigen::Vector3d tohorizontal( Eigen::Vector3d v) {
  Eigen::Vector3d h(v(0), v(1), 0);
  return h;
}

Eigen::Quaterniond toEigen(const geometry_msgs::Quaternion &p) {
  Eigen::Quaterniond q4(p.w ,p.x, p.y, p.z);
    return q4;
}

Eigen::Vector3d tovertical( Eigen::Vector3d h) {
  Eigen::Vector3d v(0, 0, h(2));
  return v;
}
inline double positive_db(double db){
  if (db > 0) return db;
  else return 0;
}

inline double dgto (double db){
  //double greater than 1
  return std::max (db ,1.0);
}

inline double sinc ( double x )
{
  if ( fabs ( x - 0.0 ) < 0.000001 )
    return 1;
  return sin ( x ) / ( x );
}
double sign ( double x ){
  if( x > 0 ) return 1.0;
  if( x < 0 ) return -1.0;
  if( x = 0 ) return 0 ;
}
geometry_msgs::Quaternion toGeometry_msgs( Eigen::Vector3d v , double w){
  geometry_msgs::Quaternion q;
  q.w = w;
  q.x = v(0);
  q.y = v(1);
  q.z = v(2);
  return q;
}
geometry_msgs::Quaternion toGeometry_msgs( Eigen::Quaterniond e){
  geometry_msgs::Quaternion q;
  q.w = e.w();
  q.x = e.x();
  q.y = e.y();
  q.z = e.z();
  return q;
}
geometry_msgs::Point toGeometry_msgs( const geometry_msgs::Vector3 &v3 ){
 geometry_msgs::Point r;
 r.x =v3.x ;
 r.y =v3.y ;
 r.z =v3.z ;
 return r;
}
geometry_msgs::Point toGeometry_msgs( const Eigen::Vector3d &v3 ){
 geometry_msgs::Point r;
 r.x =v3(0) ;
 r.y =v3(1) ;
 r.z =v3(2) ;
 return r;
}
geometry_msgs::PoseStamped toGeometry_msgs( const Eigen::Vector3d &v3 , const Eigen::Quaterniond &q4 ){
 geometry_msgs::PoseStamped r;
 r.pose.position.x =v3(0) ;
 r.pose.position.y =v3(1) ;
 r.pose.position.z =v3(2) ;
 r.pose.orientation.x = q4.x();
 r.pose.orientation.y = q4.y();
 r.pose.orientation.z = q4.z();
 r.pose.orientation.w = q4.w();
 return r;
}
inline Eigen::Vector3d toEigen(const geometry_msgs::Vector3 &v3) {
  Eigen::Vector3d ev3(v3.x, v3.y, v3.z);
  return ev3;
}
inline geometry_msgs::Vector3 toVector3(const Eigen::Vector3d &ev3) {
  geometry_msgs::Vector3 v3;
  v3.x = ev3(0);
  v3.y = ev3(1);
  v3.z = ev3(2);
  return v3;
}
inline geometry_msgs::Point toVector3(const geometry_msgs::Vector3 &v3) {
  geometry_msgs::Point p3;
  p3.x = v3.x;
  p3.y = v3.y;
  p3.z = v3.z;
  return p3;
}


Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}
double H2Vratio(Eigen::Vector3d v){
  return fabs(tohorizontal(v).norm()/v(2));
}
double H2VCoeff(Eigen::Vector3d v){
  return (-0.5 / log(H2Vratio(v)+1.8) + 0.5);
} 
double findLastAverage(std::deque<double>& signal, int n)
{ 
  if (signal.size() < n) return 1000;
  else{
    double sum = 0;
    for (int i = 0; i < n; i++)
  {
    sum+= signal.at(signal.size()-1-i);

  }
  return sum/n;
  }
}
double findLastVariance(std::deque<double>& signal, int n , double mean )
{ 
  if (signal.size() < n) return 1000;
  else{
    double sum = 0;
    for (int i = 0; i < n; i++)
  {
    sum+= (signal.at(signal.size()-1-i) - mean)* (signal.at(signal.size()-1-i) - mean);
  }
  return sqrt(sum/n);
  }
}
int findPeak(std::deque<double>& signal)
{ 
  if (signal.size() < 21) return -2;
  else{
  bool min = true ,max =true ;

  for (int i = 0; i < 21; i++)
  {
    if (signal[i] < signal[10])
    {
      min = false ;
    }
    if (signal[i] > signal[10])
    {
      max = false ;
    }
  }
  signal.pop_front();
  if(min == true) return -1 ;
  else if(max == true) return 1 ;
  else return 0 ;
  }
}

Eigen::Vector3d reScaleMax(Eigen::Vector3d v, double nml){
return v / std::max(v.norm()/nml,1.0);
}
double reScaleMax(double v, double nml){
return std::max(v , nml);
}
void integral_handle(Eigen::Vector3d &integral, Eigen::Vector3d error, double dt, Eigen::Vector3d ki_coeff ,double rescale){
if(dt > 0.05) dt =0;
Eigen::Vector3d integral_update = dt * ki_coeff.cwiseProduct(error);
integral += integral_update;
integral = reScaleMax( integral, rescale);
}

void integral_handle(Eigen::Vector3d &integral, Eigen::Vector3d error, double dt, double ki_coeff ,double turn_head_coeff ,double rescale){

Eigen::Vector3d weight = integral.cwiseProduct(error);
for(int i =0; i< 3 ; i++){
if(weight(i) >= 0 || fabs(error(i)) <0.1 ) weight(i) = ki_coeff;
else weight(i) = ki_coeff * turn_head_coeff;}
if(dt > 0.05) dt =0;
Eigen::Vector3d integral_update = dt * weight.cwiseProduct(error);
integral += integral_update;
integral = reScaleMax( integral, rescale);
}
void integral_handle(double &integral, double error, double dt, double ki_coeff ,double turn_head_coeff ,double rescale){
double weight = integral *error ;
if(weight >= 0 || fabs(error) < 0.1 ) weight = ki_coeff; 
else weight = ki_coeff * turn_head_coeff;
if(dt > 0.05) dt =0;
double integral_update = dt * weight * error;
integral += integral_update;
integral = integral / std::max(1.0,fabs(integral)/rescale);
// ROS_INFO_STREAM("integral " << integral << " " << fabs(integral)/rescale << " = " << std::max(1.0,fabs(integral)/rescale));
}
void discrete_integral_handle(double &integral, double error, double dt, double ki_coeff ,double turn_head_coeff ,double rescale){
double weight = integral *error ;
if(weight >= 0|| fabs(error) < 0.1 ) weight = ki_coeff; 
else weight = ki_coeff * turn_head_coeff;
if(dt > 0.05) dt =0;
double integral_update = dt * weight * error;
integral += integral_update;
integral = integral / std::max(1.0,fabs(integral)/rescale);
}


Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
  Eigen::Matrix3d rotmat;
  rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
      2 * q(0) * q(2) + 2 * q(1) * q(3),

      2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
      2 * q(2) * q(3) - 2 * q(0) * q(1),

      2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
      q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
  return rotmat;
}

Eigen::Vector4d rot2Quaternion(const Eigen::Matrix3d &R) {
  Eigen::Vector4d quat;
  double tr = R.trace();
  if (tr > 0.0) {
    double S = sqrt(tr + 1.0) * 2.0;  // S=4*qw
    quat(0) = 0.25 * S;
    quat(1) = (R(2, 1) - R(1, 2)) / S;
    quat(2) = (R(0, 2) - R(2, 0)) / S;
    quat(3) = (R(1, 0) - R(0, 1)) / S;
  } else if ((R(0, 0) > R(1, 1)) & (R(0, 0) > R(2, 2))) {
    double S = sqrt(1.0 + R(0, 0) - R(1, 1) - R(2, 2)) * 2.0;  // S=4*qx
    quat(0) = (R(2, 1) - R(1, 2)) / S;
    quat(1) = 0.25 * S;
    quat(2) = (R(0, 1) + R(1, 0)) / S;
    quat(3) = (R(0, 2) + R(2, 0)) / S;
  } else if (R(1, 1) > R(2, 2)) {
    double S = sqrt(1.0 + R(1, 1) - R(0, 0) - R(2, 2)) * 2.0;  // S=4*qy
    quat(0) = (R(0, 2) - R(2, 0)) / S;
    quat(1) = (R(0, 1) + R(1, 0)) / S;
    quat(2) = 0.25 * S;
    quat(3) = (R(1, 2) + R(2, 1)) / S;
  } else {
    double S = sqrt(1.0 + R(2, 2) - R(0, 0) - R(1, 1)) * 2.0;  // S=4*qz
    quat(0) = (R(1, 0) - R(0, 1)) / S;
    quat(1) = (R(0, 2) + R(2, 0)) / S;
    quat(2) = (R(1, 2) + R(2, 1)) / S;
    quat(3) = 0.25 * S;
  }
  return quat;
}

class Gauss_pdf{
private: 

float mean = 0 ;
float sigma = 0.75;
constexpr static const float inv_sqrt_2pi = 0.3989422804014327;

public:
Gauss_pdf() {}
Gauss_pdf(float mean_ , float sigma_){
  mean = mean_;
  sigma = sigma_;
}
~Gauss_pdf(){}

float normal_pdf (float x)
{
    
    float a = (x - mean) / sigma;

    return inv_sqrt_2pi / sigma * std::exp(-0.5f * a * a);
}
};

#endif
