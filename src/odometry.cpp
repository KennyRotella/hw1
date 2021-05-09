#include <math.h>
#include "ros/ros.h"
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>

#include "hw1/SetOdometry.h"
#include <hw1/integration_methodConfig.h>
#include <hw1/Odom.h>

int method = 1;

ros::Publisher pub;
ros::Publisher pub2;
double x = 0.0;
double y = 0.0;
double th = 0.0;

double last_time = 0;

bool set_odometry(hw1::SetOdometry::Request  &req,
                  hw1::SetOdometry::Response &res)
{
  ROS_INFO("request to reset odometry: x=%f, y=%f, th=%f", (float)req.x, (float)req.y, (float)req.th);
  x = (float)req.x;
  y = (float)req.y;
  th = (float)req.th;
  return true;
}

void callback1(const geometry_msgs::TwistStamped::ConstPtr& msg) {

  double current_time = double(msg->header.stamp.sec) + double(msg->header.stamp.nsec)*1e-9;

  if(last_time == 0){
    last_time = current_time;
    return;
  }

  double dt = current_time - last_time;
  last_time = current_time;

  // with respect robot reference frame (right hand rule):
  //  x: forward
  //  y: left
  //  z: upward
  double vx = msg->twist.linear.x;
  double vy = 0;
  double vth = msg->twist.angular.z;

  double delta_x, delta_y, delta_th;
  
  if(method == 0){
    delta_x = vx * cos(th) * dt;
    delta_y = vx * sin(th) * dt;
    delta_th = vth * dt;
  } else {
    delta_x = vx * cos(th+vth*dt/2) * dt;
    delta_y = vx * sin(th+vth*dt/2) * dt;
    delta_th = vth * dt;
  }

  x += delta_x;
  y += delta_y;
  th += delta_th;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

  hw1::Odom odom;
  odom.method.data = method == 0 ? "euler" : "rk";
  odom.odom.header = msg->header;
  odom.odom.header.frame_id = "odom";

  odom.odom.pose.pose.position.x = x;
  odom.odom.pose.pose.position.y = y;
  odom.odom.pose.pose.position.z = 0.0;
  odom.odom.pose.pose.orientation = odom_quat;

  odom.odom.twist.twist.linear.x = vx * cos(th);
  odom.odom.twist.twist.linear.y = vx * sin(th);
  odom.odom.twist.twist.angular.z = vth;

  pub.publish(odom);
  pub2.publish(odom.odom);
}

void callback2(hw1::integration_methodConfig &config, uint32_t level) {
  ROS_INFO("Reconfigure Request: %s", config.integration_method == 0 ? "Euler" : "Runge Kutta");

  method = config.integration_method;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odometry");

  ros::NodeHandle n;

  // initial pose from param
  std::string str;
  n.getParam("/initial_pose", str);

  std::vector<float> array;
  std::stringstream ss(str);

  float temp;
  while (ss >> temp)
    array.push_back(temp);

  x = array[0];
  y = array[1];
  th = array[2];
  ROS_INFO("Initial pose: x(%f), y(%f), th(%f)", x, y, th);

  pub = n.advertise<hw1::Odom>("/custom_msg", 1);
  pub2 = n.advertise<nav_msgs::Odometry>("/odom_approx", 1);

  ros::Subscriber sub = n.subscribe("scout_speeds", 1, callback1);

  dynamic_reconfigure::Server<hw1::integration_methodConfig> server;
  dynamic_reconfigure::Server<hw1::integration_methodConfig>::CallbackType f;

  f = boost::bind(&callback2, _1, _2);
  server.setCallback(f);

  ros::ServiceServer service = n.advertiseService("set_odometry", set_odometry);
  ROS_INFO("Service to set odometry ready.");

  ros::spin();

  return 0;
}