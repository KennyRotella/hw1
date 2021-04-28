#include "ros/ros.h"
#include "robotics_hw1/MotorSpeed.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <math.h>
#include <nav_msgs/Odometry.h>


float radius = 0.1575;
float gear_ratio = 0;
int count = 0;

void callback(const robotics_hw1::MotorSpeedConstPtr& fl,
              const robotics_hw1::MotorSpeedConstPtr& fr,
              const robotics_hw1::MotorSpeedConstPtr& rl,
              const robotics_hw1::MotorSpeedConstPtr& rr,
              const nav_msgs::OdometryConstPtr& od) {
  float rpm_left = (fl->rpm+rl->rpm)/2;
  float rpm_right = (fr->rpm+rr->rpm)/2;

  float vel_left = -radius * (2 * M_PI * rpm_left) / 60;
  float vel_right = radius * (2 * M_PI * rpm_right) / 60;

  float velx = (vel_left + vel_right) / 2;
  float velx_gt = od->twist.twist.linear.x;
  float gr = velx / velx_gt;
  if(gr > 35 && gr < 40){
    gear_ratio += gr;
    count++;
    ROS_INFO ("gear ratio: 1/(%f)", gear_ratio / count);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "subscriber_sync");

  ros::NodeHandle n;

  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub1(n, "motor_speed_fl", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub2(n, "motor_speed_fr", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub3(n, "motor_speed_rl", 1);
  message_filters::Subscriber<robotics_hw1::MotorSpeed> sub4(n, "motor_speed_rr", 1);
  message_filters::Subscriber<nav_msgs::Odometry> sub5(n, "scout_odom", 1);
  
  typedef message_filters::sync_policies::
    ExactTime<robotics_hw1::MotorSpeed,
              robotics_hw1::MotorSpeed,
              robotics_hw1::MotorSpeed,
              robotics_hw1::MotorSpeed,
              nav_msgs::Odometry> MySyncPolicy;
//  typedef message_filters::sync_policies
//      ::ApproximateTime<robotics_hw1::MotorSpeed, robotics_hw1::MotorSpeed> MySyncPolicy;
  
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3, sub4, sub5);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5));

  ros::spin();

  return 0;
}