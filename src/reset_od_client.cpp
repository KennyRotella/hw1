#include "ros/ros.h"
#include "hw1/SetOdometry.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "reset_odometry_client");

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hw1::SetOdometry>("set_odometry");
  hw1::SetOdometry srv;
  srv.request.x = 0.0;
  srv.request.y = 0.0;
  srv.request.th = 0.0;

  if (client.call(srv))
  {
    ROS_INFO("Service set_odometry done");
  }
  else
  {
    ROS_ERROR("Failed to call service set_odometry");
    return 1;
  }

  return 0;
}