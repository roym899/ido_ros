#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

// TODO make class instead of globals
nav_msgs::OccupancyGrid occupancy_grid;
const size_t WIDTH = 100;
const size_t HEIGHT = 100;
const double CELL_SIZE = 0.05;
const int8_t INIT_VALUE = 10; // -128 == 0.0; 127 == 1.0

int8_t& at(nav_msgs::OccupancyGrid& occupancy_grid, int x, int y)
{
  return occupancy_grid.data[y * occupancy_grid.info.width + x];
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  ROS_INFO(
      "I heard: angle min %f angle max %f angle increment %f",
      msg->angle_min,
      msg->angle_max,
      msg->angle_increment);
}

void reset_occupancy_grid(const size_t width, const size_t height, const int8_t init_value)
{
  occupancy_grid.header.frame_id = "map";
  occupancy_grid.info.height = height;
  occupancy_grid.info.width = width;

  const size_t num_cells = width * height;
  occupancy_grid.data = std::vector<int8_t>(num_cells, init_value);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ido_node");

  ros::NodeHandle nh_priv("~");
  ros::NodeHandle nh;

  reset_occupancy_grid(WIDTH, HEIGHT, INIT_VALUE);

  ros::Publisher occ_pub = nh_priv.advertise<nav_msgs::OccupancyGrid>("occupancy", 1000);
  ros::Subscriber sub = nh.subscribe("scan", 1000, scanCallback);

  ROS_INFO("%d\n", b);
  ROS_INFO("%d\n", 1);

  // TODO support pose topic Pose2DStamped (custom message?) or PoseStamped (?)

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}
