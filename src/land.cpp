#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

Quaterniond rotationToQuaternion(const Vector3d& u, const Vector3d& v)
{
    Vector3d axis = u.cross(v);
    double angle = acos(u.dot(v) / (u.norm() * v.norm()));
    return Quaterniond(AngleAxisd(angle, axis.normalized()));
}

int main()
{
    // Initialize the forward and up direction vectors of the first pose
    Eigen::Vector3d forward1(1, 0, 0);
    Eigen::Vector3d upwards(0,0,1);
    Eigen::Vector3d up1(1, 0, 0);   // Convert the combined rotation matrix to a quaternion

    
    Eigen::Vector3d forward2(0, 0, -1);
    Eigen::Vector3d up2(0, -1, 0);

  
   
      Eigen::Quaterniond q1 = rotationToQuaternion(forward1,forward2);
        Eigen::Quaterniond q2 = rotationToQuaternion(up1,up2);
std::cout << "Quaternion:1 " << q1.w() << " " << q1.x() << " " << q1.y() << " " << q1.z() << std::endl;
std::cout << "Quaternion:2 " << q2.w() << " " << q2.x() << " " << q2.y() << " " << q2.z() << std::endl;

  Eigen::Quaterniond q = q2 * q1;
  q.normalize();
    // Output the resulting quaternion
    std::cout << "Quaternion: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    Vector3d forwarde = q * forward1 ;
    Vector3d upwarde = q * upwards ;
    std::cout << forwarde(0) <<" "<< forwarde(1) << " " <<forwarde(2) << " \n";
    std::cout << upwarde(0) <<" "<< upwarde(1) << " " <<upwarde(2) << " \n";
    return 0;
}
// #include "ros/ros.h"
// #include <cstdlib>
// #include "geometric_controller/geometric_controller.h"

// int main(int argc, char **argv)
// {
//   ros::init(argc, argv, "land");
//   ros::NodeHandle n;
//   ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("land");
//   std_srvs::SetBool::Request land_rq;
//   std_srvs::SetBool::Response land_rs;
//   if(argc == 2)
//   land_rq.data = std::atoi(argv[1]);
//   else land_rq.data = true;
//   bool success = client.call(land_rq,land_rs);
//   if(success && argv[1] == "0") 
//   ROS_INFO_STREAM("Marker not found");
//   else if (success)
//   ROS_INFO_STREAM("Landing");
//   else 
//   ROS_INFO_STREAM("RIP DRONE 6/2022"); 
//   double ta =3.0/100;
//   for (int i = 0; i <100 ;i++){
//   double tc = pow((0.8 * sinc (2.0 * ta*i -3.0)-0.025),2);
//   ROS_INFO_STREAM(i*ta<<" "<<tc); }
//   return 0;
// }