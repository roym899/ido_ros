#include <ido_ros/ido_node.h>

#include <omp.h>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <thread>

// TODO make these proper parameters
const float WIDTH = 9;  // width of map in meters
const float HEIGHT = 9; // height of map in meters
const size_t CELLS_PER_METER = 15;
const float PRIOR_PROB = 0.5;
const float METER_PER_PREDICTIONSTEP = 0.1;  // i.e., computed from max velocity
const float NUM_PREDICTION_STEPS = 2;
const bool SKIP_NORETURN = false;
const float RANGE_NORETURN = 4.0; // only used if SKIP_NORETURN == False
const size_t NUM_THREADS = 4;
const size_t MAGIC_CON = 8;

void fix_angle(float& angle)
{
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0)
        angle += M_PI;
    else
        angle -= M_PI;
}

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
            msg.data[i * cols + j] = std::round((*this)(i, j) * 100.0);
        }
    }

    return msg;
}

void IDONode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    auto start = std::chrono::high_resolution_clock::now();

    // 1. prediction step
    // 1.1. prediction on prob using 2D convolution
    auto start_p = std::chrono::high_resolution_clock::now();
    for(int i=0; i<NUM_PREDICTION_STEPS; ++i)
        probs_ = predictMotion(probs_);
    std::chrono::duration<float> time_prediction = std::chrono::high_resolution_clock::now() - start_p;

    // 1.2. probs to log odds
    LogOddsGrid log_odds = probs_.toLogOdds();

    // 2. update step
    // 2.1. update log odds based on scan using ray casting
    auto start_u = std::chrono::high_resolution_clock::now();
    log_odds.insertScan(*msg);
    std::chrono::duration<float> time_update = std::chrono::high_resolution_clock::now() - start;

    // 2.2. convert log odds to probabilities
    probs_ = log_odds.toProbs();
    // 2.3. convert to ROS message
    auto occupancy_grid_msg = probs_.toOccupancyGridMsg();
    // 2.4. publish ROS Message
    occ_pub_.publish(occupancy_grid_msg);

    std::chrono::duration<float> time = std::chrono::high_resolution_clock::now() - start;

    ROS_INFO(
        "Prediction: %f Update: %f Total: %f ",
        time_prediction.count(),
        time_update.count(),
        time.count());
}

void LogOddsGrid::insertScan(const sensor_msgs::LaserScan& msg, const geometry_msgs::Pose2D& pose)
{
    float laser_angle = msg.angle_min;
    for (const auto& range : msg.ranges) {
        // offset the particle based on the lidar position
        fix_angle(laser_angle);
        if (range != 0.0)
            insertRay(pose.x, pose.y, laser_angle, range);
        else if (not SKIP_NORETURN)
            insertRay(pose.x, pose.y, laser_angle, RANGE_NORETURN, true);
        laser_angle += msg.angle_increment;
    }
}

void LogOddsGrid::insertRay(float x, float y, const float angle, const float range, const bool no_return)
{
    // TODO check point to cell and ray cast is all correct
    x = (x + WIDTH / 2) * CELLS_PER_METER;
    y = (y + HEIGHT / 2) * CELLS_PER_METER;

    float x_start = x;
    float y_start = y;

    int x_discrete = std::floor(x);
    int y_discrete = std::floor(y);

    float v_x = cos(angle);
    float v_y = sin(angle);

    int step_x = std::abs(angle) <= M_PI / 2 ? 1 : -1;
    int step_y = angle > 0 ? 1 : -1;

    // given the ray (x,y)+t*(vx,vy), check for which t the next cell in x and y direction is reached
    float x_boundary = step_x > 0 ? std::ceil(x + 0.5) : std::floor(x);
    float y_boundary = step_y > 0 ? std::ceil(y + 0.5) : std::floor(y);
    float t_max_x = (x_boundary - x) / v_x;
    float t_max_y = (y_boundary - y) / v_y;

    // how far to increase t in x and y to go through a whole grid cell
    float t_delta_x = std::abs(1 / v_x);
    float t_delta_y = std::abs(1 / v_y);

    // iterate until out of map or range reached
    while (std::sqrt(std::pow(x_discrete + 0.5 - x, 2) + std::pow(y_discrete + 0.5 - y, 2)) / CELLS_PER_METER < range - 0.5 / CELLS_PER_METER) {
        if (x_discrete < 0 || x_discrete > cols) {
            break;
        } else if (y_discrete < 0 || y_discrete > rows) {
            break;
        } else if ((*this)(y_discrete, x_discrete) >= -15) {
            (*this)(y_discrete, x_discrete) -= 5;
        }

        // go to next cell
        if (t_max_x < t_max_y) {
            t_max_x = t_max_x + t_delta_x;
            x_discrete = x_discrete + step_x;
        } else {
            t_max_y = t_max_y + t_delta_y;
            y_discrete = y_discrete + step_y;
        }
    }
    if (no_return) // no return -> only clear, does not increase occupancy
        return;
    else if (x_discrete < 0 || x_discrete > cols) {
        return;
    } else if (y_discrete < 0 || y_discrete > rows) {
        return;
    }
    if ((*this)(y_discrete, x_discrete) <= 15) {
        (*this)(y_discrete, x_discrete) += 5;
    }
}

void IDONode::initKernel()
{
    const float cells_per_step = METER_PER_PREDICTIONSTEP * CELLS_PER_METER;
    const size_t kernel_size = std::floor(cells_per_step);
    kernel_ = Matrix2D(2 * kernel_size + 1, 2 * kernel_size + 1, 0.0);
    const int center = kernel_size;
    size_t counter = 0;
    for (int i = 0; i < kernel_.rows; ++i) {
        for (int j = 0; j < kernel_.cols; ++j) {
            float cell_distance = std::sqrt(std::pow(i - center, 2) + std::pow(j - center, 2));
            if (cell_distance <= cells_per_step)
                ++counter;
        }
    }
    const float value = 1.0 / counter;
    for (int i = 0; i < kernel_.rows; ++i) {
        for (int j = 0; j < kernel_.cols; ++j) {
            float cell_distance = std::sqrt(std::pow(i - center, 2) + std::pow(j - center, 2));
            if (cell_distance <= cells_per_step)
                kernel_(i, j) = value;
            std::cout << kernel_(i, j) << " ";
        }
        std::cout << std::endl;
    }
    kernel_center_ = center;
    std::cout << kernel_center_ << " " << kernel_.rows << std::endl;
}

ProbabilityGrid IDONode::predictMotion(const ProbabilityGrid& occ_probs) const
{
    ProbabilityGrid prediction(occ_probs.rows, occ_probs.cols, PRIOR_PROB);
    omp_set_num_threads(NUM_THREADS);

#pragma omp parallel for
    for (size_t i = kernel_center_; i < occ_probs.rows - kernel_center_; ++i) {
        for (size_t j = kernel_center_; j < occ_probs.cols - kernel_center_ - MAGIC_CON + 1; j += MAGIC_CON) {
            for (size_t offset = 0; offset < MAGIC_CON; ++offset)
                prediction(i, j + offset) = 0;
            for (size_t k = 0; k < kernel_.rows; ++k) {
                for (size_t l = 0; l < kernel_.cols; ++l) {
                    for (size_t offset = 0; offset < MAGIC_CON; ++offset)
                        prediction(i, j + offset) += 
                            occ_probs(i - kernel_center_ + k, 
                                      j + offset - kernel_center_ + l) * kernel_(k, l);
                }
            }
        }
    }
    return prediction;
}

ProbabilityGrid LogOddsGrid::toProbs() const
{
    ProbabilityGrid probs(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            float odds = std::exp((*this)(i, j));
            probs(i, j) = odds / (odds + 1);
        }
    }
    return probs;
}

LogOddsGrid ProbabilityGrid::toLogOdds() const
{
    LogOddsGrid log_odds(rows, cols);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            float odds = (*this)(i, j) / (1 - (*this)(i, j));
            log_odds(i, j) = std::log(odds);
        }
    }
    return log_odds;
}

IDONode::IDONode()
    : nh_priv_("~")
    , probs_(HEIGHT * CELLS_PER_METER, WIDTH * CELLS_PER_METER, PRIOR_PROB)
    , kernel_(0, 0, 0)
{
    occ_pub_ = nh_priv_.advertise<nav_msgs::OccupancyGrid>("occupancy", 1000);
    scan_sub_ = nh_.subscribe("/scan", 1, &IDONode::scanCallback, this);
    initKernel();
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
