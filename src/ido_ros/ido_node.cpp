#include <ido_ros/ido_node.h>

#include <sstream>

extern ProbabilityGrid probs;

nav_msgs::OccupancyGrid ProbabilityGrid::toOccupancyGrid() const
{
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
    ROS_INFO("receiving scan");
    // 1. prediction step
    // 1.1. TODO prediction on prob using 2D convolution
    // 1.2. probs to log odds
    LogOddsGrid log_odds(probs);

    // 2. update step
    // 2.1. update log odds based on scan using ray casting
    log_odds.insertScan(*msg);
    // 2.2. TODO convert log odds to probabilities and publish ROS message based on that
}

IDONode::IDONode()
    : nh_priv_("~")
{
    occ_pub_ = nh_priv_.advertise<nav_msgs::OccupancyGrid>("occupancy", 1000);
    scan_sub_ = nh_.subscribe("/scan", 1000, scanCallback);
}

int IDONode::run() {
    // TODO support pose topic Pose2DStamped (custom message, nav 2d package?),
    //  or use 3D PoseStamped (?)

    ros::Rate loop_rate(100);

    ros::spin();
    return 0;
}

// TODO make class instead of globals
nav_msgs::OccupancyGrid occupancy_grid;
const double WIDTH = 7;       // width of map in meters
const double HEIGHT = 7;      // height of map in meters
const size_t RESOLUTION = 15; // cells per meter
const double PRIOR_PROB = 0.5;

ProbabilityGrid probs(HEIGHT* RESOLUTION, WIDTH* RESOLUTION, PRIOR_PROB);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ido_node");

    IDONode node;

    return node.run();
}
