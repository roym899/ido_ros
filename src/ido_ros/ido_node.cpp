#include <ido_ros/ido_node.h>

#include <chrono>
#include <cmath>
#include <sstream>

// TODO make these proper parameters
const double WIDTH = 7;  // width of map in meters
const double HEIGHT = 7; // height of map in meters
const size_t CELLS_PER_METER = 15;
const double PRIOR_PROB = 0.5;

nav_msgs::OccupancyGrid ProbabilityGrid::toOccupancyGridMsg() const
{
    nav_msgs::OccupancyGrid msg;

    // 1. set header, width, height
    msg.header.frame_id = "base_scan";
    msg.header.stamp = ros::Time::now();
    msg.info.width = cols;
    msg.info.height = rows;
    msg.info.resolution = 1.0 / CELLS_PER_METER;

    // center at 0, 0 for now
    msg.info.origin.position.x = -0.5 * WIDTH;
    msg.info.origin.position.y = -0.5 * HEIGHT;

    // 2. convert to int8 (0-100) range, see docs of OccupancyGrid
    msg.data = std::vector<int8_t>(rows * cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            msg.data[i * cols + j] = std::round((*this)(i, j) / 100.0);
        }
    }

    return msg;
}

void IDONode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ROS_INFO("receiving scan");
    auto start = std::chrono::high_resolution_clock::now();

    // 1. prediction step
    // 1.1. TODO prediction on prob using 2D convolution
    // 1.2. probs to log odds
    LogOddsGrid log_odds = probs_.toLogOdds();

    // 2. update step
    // 2.1. update log odds based on scan using ray casting
    log_odds.insertScan(*msg);
    // 2.2. convert log odds to probabilities
    probs_ = log_odds.toProbs();
    // 2.3. convert to ROS message
    auto occupancy_grid_msg = probs_.toOccupancyGridMsg();
    // 2.4. publish ROS Message
    occ_pub_.publish(occupancy_grid_msg);

    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> a = end - start;

    ROS_INFO("published scan (callback took %fs)", a.count());
}

void LogOddsGrid::insertScan(const sensor_msgs::LaserScan& msg, const geometry_msgs::Pose2D& pose)
{
    // TODO insert scan in log odds using ray casting
}

ProbabilityGrid LogOddsGrid::toProbs() const
{
    ProbabilityGrid probs(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            double prob = std::exp((*this)(i, j));
            probs(i, j) = 1 / (1 - prob);
        }
    }
    return probs;
}

LogOddsGrid ProbabilityGrid::toLogOdds() const
{
    LogOddsGrid log_odds(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            double odds = (*this)(i, j) / (1 - (*this)(i, j));
            log_odds(i, j) = std::log(odds);
        }
    }
    return log_odds;
}

IDONode::IDONode()
    : nh_priv_("~")
    , probs_(HEIGHT * CELLS_PER_METER, WIDTH * CELLS_PER_METER, PRIOR_PROB)
{
    occ_pub_ = nh_priv_.advertise<nav_msgs::OccupancyGrid>("occupancy", 1000);
    scan_sub_ = nh_.subscribe("/scan", 1000, &IDONode::scanCallback, this);
}

int IDONode::run()
{
    // TODO support pose topic Pose2DStamped (custom message, nav 2d package?),
    //  or use 3D PoseStamped (?)

    ros::Rate loop_rate(100);

    ros::spin();
    return 0;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ido_node");

    IDONode node;

    return node.run();
}
