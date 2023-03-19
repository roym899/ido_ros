#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <sstream>

// TODO make class instead of globals
nav_msgs::OccupancyGrid occupancy_grid;
const double WIDTH = 7;  // width of map in meters
const double HEIGHT = 7;  // height of map in meters
const size_t RESOLUTION = 15;  // cells per meter
const double PRIOR_PROB = 0.5;

struct ProbabilityGrid {
  ProbabilityGrid(size_t width, size_t height, double a=0.5)
    : probabilities(width * height, a), width(width), height(height) {}
  std::vector<double> probabilities;
  size_t width;
  size_t height;
  nav_msgs::OccupancyGrid toOccupancyGrid() const;
};

nav_msgs::OccupancyGrid ProbabilityGrid::toOccupancyGrid() const {
  // TODO implement conversion
  //  1. set header, width, height
  //  2. convert to int8 (0-100) range, see docs of OccupancyGrid
  return nav_msgs::OccupancyGrid();
}

ProbabilityGrid cur_map(WIDTH * RESOLUTION, HEIGHT * RESOLUTION, PRIOR_PROB);

int8_t& at(nav_msgs::OccupancyGrid& occupancy_grid, int x, int y)
{
  return occupancy_grid.data[y * occupancy_grid.info.width + x];
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // TODO prediction step
  //  1. prediction on prob using 2D convolution
  //  2. probs to log odds

  // TODO update step 
  //  1. update log odds through addition with ray casting
  //  2. convert log odds to probabilities and publish ROS message based on that
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ido_node");

  ros::NodeHandle nh_priv("~");
  ros::NodeHandle nh;

  ros::Publisher occ_pub = nh_priv.advertise<nav_msgs::OccupancyGrid>("occupancy", 1000);
  ros::Subscriber sub = nh.subscribe("scan", 1000, scanCallback);

  // TODO support pose topic Pose2DStamped (custom message?) or PoseStamped (?)

  ros::Rate loop_rate(10);

  ros::spin();

  return 0;
}
