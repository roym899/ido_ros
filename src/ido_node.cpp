#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

// TODO make class for node

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO(
      "I heard: angle min %f angle max %f angle increment %f",
      msg->angle_min,
      msg->angle_max,
      msg->angle_increment);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ido_node");

  ros::NodeHandle nh_priv("~");
  ros::NodeHandle nh;

  ros::Publisher occ_pub = nh_priv.advertise<nav_msgs::OccupancyGrid>("occupancy", 1000);
  ros::Subscriber sub = nh.subscribe("scan", 1000, scanCallback);

  ros::Rate loop_rate(10);

  ros::spin();
  /* int count = 0; */
  /* while (ros::ok()) { */
  /*   std_msgs::String msg; */

  /*   std::stringstream ss; */
  /*   ss << "hello world " << count; */
  /*   msg.data = ss.str(); */

  /*   ROS_INFO("%s", msg.data.c_str()); */

  /*   chatter_pub.publish(msg); */

  /*   ros::spinOnce(); */

  /*   loop_rate.sleep(); */
  /*   ++count; */
  /* } */

  return 0;
}
