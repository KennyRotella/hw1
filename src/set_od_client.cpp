#include "ros/ros.h"
#include "hw1/SetOdometry.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "set_odometry_client");
  if (argc != 4)
  {
    ROS_INFO("usage: reset_od_client X Y THETA");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<hw1::SetOdometry>("set_odometry");
  hw1::SetOdometry srv;
  srv.request.x = std::stof(argv[1]);
  srv.request.y = std::stof(argv[2]);
  srv.request.th = std::stof(argv[3]);

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