#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class tf_sub_pub
{
public:
  tf_sub_pub() {
    sub = n.subscribe("odom_approx", 1, &tf_sub_pub::callback, this);
  }

  void callback(const nav_msgs::Odometry::ConstPtr& msg){
    transformStamped.header = msg->header;
    transformStamped.child_frame_id = "robot";
    transformStamped.transform.translation.x = msg->pose.pose.position.x;
    transformStamped.transform.translation.y = msg->pose.pose.position.y;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation = msg->pose.pose.orientation;

    br.sendTransform(transformStamped);
  }

private:
  ros::NodeHandle n; 
  tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;
  ros::Subscriber sub;
};


int main(int argc, char **argv) {
  ros::init(argc, argv, "odometry_tf");
  tf_sub_pub my_tf_sub_pub;
  ros::spin();
  return 0;
}
