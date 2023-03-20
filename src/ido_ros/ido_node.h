#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

struct Matrix2D {
    Matrix2D(size_t rows, size_t cols, double value = 0.0)
        : data(rows * cols, value)
        , rows(rows)
        , cols(cols)
    {
    }
    std::vector<double> data;
    const size_t rows;
    const size_t cols;
};

struct ProbabilityGrid {
    ProbabilityGrid(size_t rows, size_t cols, double prior = 0.5)
        : probabilities(rows, cols, prior)
    {
    }
    Matrix2D probabilities;
    void predictTransitions(const Matrix2D& kernel);
    nav_msgs::OccupancyGrid toOccupancyGrid() const;
};

struct LogOddsGrid {
    LogOddsGrid(size_t rows, size_t cols, double prior = 0.0)
        : log_odds(rows, cols, prior)
    {
    }
    LogOddsGrid(const ProbabilityGrid& probability_grid)
        : log_odds(probability_grid.probabilities.rows, probability_grid.probabilities.cols)
    {
        // NOTE would be better to somehow skip value initialization here
        // TODO convert probability grid to log odds grid
    }
    void insertScan(const sensor_msgs::LaserScan& msg, const geometry_msgs::Pose2D& pose = geometry_msgs::Pose2D())
    {
        // TODO insert the laser scan at the specified pose
    }
    Matrix2D log_odds;
};

class IDONode {
public:
    IDONode();
    int run();

private:
    ros::NodeHandle nh_priv_;
    ros::NodeHandle nh_;

    ros::Publisher occ_pub_;
    ros::Subscriber scan_sub_;
};
