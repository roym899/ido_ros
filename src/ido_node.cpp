#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose2D.h"

#include <sstream>

struct Matrix2D {
  Matrix2D(size_t rows, size_t cols, double value=0.0) 
    : data(rows * cols, value), rows(rows), cols(cols) {}
  std::vector<double> data;
  const size_t rows;
  const size_t cols;
};

struct ProbabilityGrid {
  ProbabilityGrid(size_t rows, size_t cols, double prior=0.5)
    : probabilities(rows, cols, prior) {}
  Matrix2D probabilities;
  void predictTransitions(const Matrix2D& kernel);
  nav_msgs::OccupancyGrid toOccupancyGrid() const;
};

extern ProbabilityGrid probs;

struct LogOddsGrid {
  LogOddsGrid(size_t rows, size_t cols, double prior=0.0)
    : log_odds(rows, cols, prior) {}
  LogOddsGrid(const ProbabilityGrid& probability_grid) 
    : log_odds(probability_grid.probabilities.rows, probability_grid.probabilities.cols) {
      // NOTE would be better to somehow skip value initialization here
      // TODO convert probability grid to log odds grid
  }
  void insertScan(const sensor_msgs::LaserScan& msg, const geometry_msgs::Pose2D& pose = geometry_msgs::Pose2D()) {
    // TODO insert the laser scan at the specified pose
  }
  Matrix2D log_odds;
};

nav_msgs::OccupancyGrid ProbabilityGrid::toOccupancyGrid() const {
  // TODO implement conversion
  //  1. set header, width, height
  //  2. convert to int8 (0-100) range, see docs of OccupancyGrid
  return nav_msgs::OccupancyGrid();
}

int8_t& at(nav_msgs::OccupancyGrid& occupancy_grid, int x, int y)
{
  return occupancy_grid.data[y * occupancy_grid.info.width + x];
}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // 1. prediction step
  // 1.1. TODO prediction on prob using 2D convolution
  // 1.2. probs to log odds
  LogOddsGrid log_odds(probs);

  // 2. update step 
  // 2.1. update log odds based on scan using ray casting
  log_odds.insertScan(*msg);
  // 2.2. TODO convert log odds to probabilities and publish ROS message based on that
}

// TODO make class instead of globals
nav_msgs::OccupancyGrid occupancy_grid;
const double WIDTH = 7;  // width of map in meters
const double HEIGHT = 7;  // height of map in meters
const size_t RESOLUTION = 15;  // cells per meter
const double PRIOR_PROB = 0.5;

ProbabilityGrid probs(HEIGHT * RESOLUTION, WIDTH * RESOLUTION, PRIOR_PROB);

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
