#include <vector>

#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class LogOddsGrid;

struct Matrix2D {
    Matrix2D(size_t rows, size_t cols, float value = 0.0)
        : data(rows * cols, value)
        , rows(rows)
        , cols(cols)
    {
    }
    std::vector<float> data;
    size_t rows;
    size_t cols;
    const float& operator()(const size_t i, const size_t j) const
    {
        return data[cols * i + j];
    };
    float& operator()(const size_t i, const size_t j)
    {
        return const_cast<float&>((*const_cast<const Matrix2D*>(this))(i, j));
    };
};

struct ProbabilityGrid : Matrix2D {
    ProbabilityGrid(size_t rows, size_t cols, float prior = 0.5)
        : Matrix2D(rows, cols, prior)
    {
    }
    void predictTransitions(const Matrix2D& kernel);
    nav_msgs::OccupancyGrid toOccupancyGridMsg() const;
    LogOddsGrid toLogOdds() const;
};

struct LogOddsGrid : Matrix2D {
    LogOddsGrid(size_t rows, size_t cols, float prior = 0.0)
        : Matrix2D(rows, cols, prior)
    {
    }
    void insertScan(const sensor_msgs::LaserScan& msg, const geometry_msgs::Pose2D& pose = geometry_msgs::Pose2D());
    void insertRay(float x, float y, float angle, float range);
    ProbabilityGrid toProbs() const;
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

    ProbabilityGrid probs_;

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
};
