#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>

float radius = 0.1575;
float real_bl = 0.583;
float gear_ratio = 1/38.210690;
float chi_param = 1.778309;
float apparent_bl = real_bl * chi_param;
ros::Publisher pub;

void callback(const robotics_hw1::MotorSpeedConstPtr& fl,
              const robotics_hw1::MotorSpeedConstPtr& fr,
              const robotics_hw1::MotorSpeedConstPtr& rl,
              const robotics_hw1::MotorSpeedConstPtr& rr) {
  float rpm_left = (fl->rpm+rl->rpm)/2;
  float rpm_right = (fr->rpm+rr->rpm)/2;

  float vel_left = -radius * gear_ratio * (2 * M_PI * rpm_left) / 60;
  float vel_right = radius * gear_ratio * (2 * M_PI * rpm_right) / 60;

  float velx = (vel_left + vel_right) / 2;
  float angular_vel = (-vel_left + vel_right)/apparent_bl;

  ROS_INFO ("vel ang: (%f) (%f)", velx, angular_vel);
  geometry_msgs::TwistStamped msg;
  msg.header = fl->header;
  msg.twist.linear.x = velx;
  msg.twist.angular.z = angular_vel;
  pub.publish(msg);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber_sync");

  ros::NodeHandle n;
  pub = n.advertise<geometry_msgs::TwistStamped>("/scout_speeds", 1);

  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(n, "motor_speed_fl", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(n, "motor_speed_fr", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(n, "motor_speed_rl", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(n, "motor_speed_rr", 1);
  
  typedef message_filters::sync_policies::
    ExactTime<robotics_hw1::MotorSpeed,
              robotics_hw1::MotorSpeed,
              robotics_hw1::MotorSpeed,
              robotics_hw1::MotorSpeed> MySyncPolicy;
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3, sub4);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4));

  ros::spin();

  return 0;
}