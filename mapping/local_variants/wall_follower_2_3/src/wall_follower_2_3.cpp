// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Original Last Modified: 2024.10.14
//
#include "wall_follower/wall_follower.hpp"

#include <memory>

#include <cmath>

#include <cstdio>
#include <stdexcept>
#include <string>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>

using namespace std::chrono_literals;

static bool g_scan_received = false;

static bool g_scan_meta_printed = false;

static double g_last_cmd_v = 0.0;
static double g_last_cmd_w = 0.0;

WallFollower::WallFollower()
: Node("wall_follower_node")
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;  // Read once when the node starts.
    auto positive = [this, &descriptor](const char * name, double default_value) {
        const double value = declare_parameter<double>(name, default_value, descriptor);
        if (!std::isfinite(value) || value <= 0.0) {
            throw std::invalid_argument(std::string(name) + " must be a finite positive number");
        }
        return value;
    };
    front_block_dist_ = positive("front_block_dist", 0.50);
    front_left_close_dist_ = positive("front_left_close_dist", 0.50);
    front_right_close_dist_ = positive("front_right_close_dist", 0.50);
    left_front_close_dist_ = positive("left_front_close_dist", 0.30);
    left_too_far_dist_ = positive("left_too_far_dist", 0.50);
    left_open_dist_ = positive("left_open_dist", 0.60);
    linear_speed_m_s_ = positive("linear_speed_m_s", 0.15);
    turn_linear_speed_m_s_ = positive("turn_linear_speed_m_s", 0.10);
    turn_speed_rad_s_ = positive("turn_speed_rad_s", 0.60);
    front_block_turn_speed_rad_s_ = positive("front_block_turn_speed_rad_s", 1.00);
    correction_turn_speed_rad_s_ = positive("correction_turn_speed_rad_s", 0.40);
    gentle_turn_speed_rad_s_ = positive("gentle_turn_speed_rad_s", 0.30);
    if (!(left_front_close_dist_ < left_too_far_dist_ && left_too_far_dist_ < left_open_dist_)) {
        throw std::invalid_argument("Expected left_front_close_dist < left_too_far_dist < left_open_dist");
    }

    for (int i = 0; i < 12; i++)
    {
        scan_data_[i] = 0.0;
    }

    robot_pose_ = 0.0;

    near_start = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    start_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("wall_follower/start_pose", latched);
    finished_pub_ = create_publisher<std_msgs::msg::Bool>("wall_follower/finished", latched);

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    scan_sub_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",

            rclcpp::SensorDataQoS(),

            std::bind(
                &WallFollower::scan_callback,
                this,
                std::placeholders::_1));

    odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",

            qos,

            std::bind(
                &WallFollower::odom_callback,
                this,
                std::placeholders::_1));

    update_timer_ =
        this->create_wall_timer(
            50ms,
            std::bind(&WallFollower::update_callback, this));

    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been terminated");
}

#define START_RANGE 0.30
#define LEAVE_RANGE 0.85

void WallFollower::odom_callback(
    const nav_msgs::msg::Odometry::SharedPtr msg)
{
    if (near_start) return;
    static bool first = true;

    static bool start_moving = true;

    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;

    m.getRPY(roll, pitch, yaw);

    robot_pose_ = yaw;

    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    if (first)
    {
        start_x = current_x;

        start_y = current_y;

        geometry_msgs::msg::PoseStamped start_pose;
        start_pose.header = msg->header;
        start_pose.pose = msg->pose.pose;
        start_pose_pub_->publish(start_pose);
        first = false;

        fprintf(
            stderr,
            "[ODOM] Start position: x=%.3f y=%.3f\n",
            start_x,
            start_y);
    }

    else if (start_moving)
    {
        if (
            std::fabs(current_x - start_x) > LEAVE_RANGE ||
            std::fabs(current_y - start_y) > LEAVE_RANGE)
        {
            start_moving = false;

            fprintf(stderr, "[ODOM] Robot has left the starting area\n");
        }
    }

    else if (
        std::fabs(current_x - start_x) < START_RANGE &&
        std::fabs(current_y - start_y) < START_RANGE)
    {
        fprintf(stderr, "[ODOM] Near start!!\n");

        near_start = true;

        std_msgs::msg::Bool finished;
        finished.data = true;
        finished_pub_->publish(finished);
    }
}

#define BEAM_WIDTH 15

void WallFollower::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    uint16_t scan_angle[12] =
    {
        0, 30, 60, 90, 120, 150,
        180, 210, 240, 270, 300, 330
    };

    if (!g_scan_meta_printed)
    {
        fprintf(
            stderr,
            "[SCAN META] ranges.size=%zu range_min=%.3f range_max=%.3f "
            "angle_min=%.3f angle_max=%.3f angle_increment=%.6f\n",
            msg->ranges.size(),
            msg->range_min,
            msg->range_max,
            msg->angle_min,
            msg->angle_max,
            msg->angle_increment);

        fprintf(
            stderr,
            "[SCAN META] IMPORTANT: this code assumes indexes 0..359 roughly map to 0..359 degrees.\n");

        g_scan_meta_printed = true;
    }

    if (msg->ranges.size() < 360)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "LaserScan has only %zu ranges; this code expects at least 360",
            msg->ranges.size());
        return;
    }

    double closest = msg->range_max;

    for (int angle = 360 - BEAM_WIDTH; angle < 360; angle++)
    {
        const float r = msg->ranges.at(angle);

        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;

        if (r < closest)
            closest = r;
    }

    for (int angle = 0; angle < BEAM_WIDTH; angle++)
    {
        const float r = msg->ranges.at(angle);

        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;

        if (r < closest)
            closest = r;
    }

    scan_data_[0] = closest;

    for (int i = 1; i < 12; i++)
    {
        closest = msg->range_max;

        for (
            int angle = scan_angle[i] - BEAM_WIDTH;
            angle < scan_angle[i] + BEAM_WIDTH;
            angle++)
        {
            const float r = msg->ranges.at(angle);

            if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
                continue;

            if (r < closest)
                closest = r;
        }

        scan_data_[i] = closest;
    }

    g_scan_received = true;
}

void WallFollower::update_cmd_vel(
    double linear,
    double angular)
{
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = linear;

    cmd_vel.angular.z = angular;

    cmd_vel_pub_->publish(cmd_vel);

    g_last_cmd_v = linear;
    g_last_cmd_w = angular;
}

void WallFollower::update_callback()
{
    if (!g_scan_received)
    {
        update_cmd_vel(0.0, 0.0);

        static bool waiting_message_printed = false;
        if (!waiting_message_printed)
        {
            fprintf(stderr, "[WAIT] No valid /scan received yet. Robot stays stopped.\n");
            waiting_message_printed = true;
        }

        return;
    }

    if (near_start)
    {
        update_cmd_vel(0.0, 0.0);

        // Keep publishing zero while the recorder receives the finish notification.
        static int stop_cycles = 0;
        if (++stop_cycles >= 20)
        {
            fprintf(stderr, "[STOP] Robot returned near start.\n");
            rclcpp::shutdown();
        }
        return;
    }

    int rule = 0;

    fprintf(
        stderr,
        "[DATA] FRONT=%.3f | FRONT_LEFT=%.3f | LEFT_FRONT=%.3f | FRONT_RIGHT=%.3f\n",
        scan_data_[FRONT],
        scan_data_[FRONT_LEFT],
        scan_data_[LEFT_FRONT],
        scan_data_[FRONT_RIGHT]);

    if (scan_data_[FRONT] < front_block_dist_)
    {
        rule = 1;
        fprintf(
            stderr,
            "[RULE 1] FRONT < %.2f -> TURN RIGHT IN PLACE\n",
            front_block_dist_);

        update_cmd_vel(0.0, -front_block_turn_speed_rad_s_);
    }

    else if (scan_data_[FRONT_LEFT] < front_left_close_dist_)
    {
        rule = 3;
        fprintf(
            stderr,
            "[RULE 3] FRONT_LEFT < %.2f -> TOO CLOSE LEFT, STEER RIGHT\n",
            front_left_close_dist_);

        update_cmd_vel(linear_speed_m_s_, -correction_turn_speed_rad_s_);
    }

    else if (scan_data_[LEFT_FRONT] < left_front_close_dist_)
    {
        fprintf(
            stderr,
            "[LEFT_FRONT CLOSE] LEFT_FRONT < %.2f -> KEEP STEERING RIGHT\n", left_front_close_dist_);

        update_cmd_vel(turn_linear_speed_m_s_, -correction_turn_speed_rad_s_);
    }

    else if (scan_data_[LEFT_FRONT] > left_open_dist_)
    {
        rule = 2;
        fprintf(
            stderr,
            "[RULE 2] LEFT_FRONT > %.2f -> LEFT WALL OPEN, TURN LEFT\n",
            left_open_dist_);

        update_cmd_vel(turn_linear_speed_m_s_, turn_speed_rad_s_);
    }

    else if (scan_data_[FRONT_RIGHT] < front_right_close_dist_)
    {
        rule = 4;
        fprintf(
            stderr,
            "[RULE 4] FRONT_RIGHT < %.2f -> TOO CLOSE RIGHT, STEER LEFT\n",
            front_right_close_dist_);

        update_cmd_vel(linear_speed_m_s_, correction_turn_speed_rad_s_);
    }

    else if (scan_data_[LEFT_FRONT] > left_too_far_dist_)
    {
        rule = 5;
        fprintf(
            stderr,
            "[RULE 5] LEFT_FRONT > %.2f -> SLIGHTLY FAR FROM LEFT, STEER LEFT\n",
            left_too_far_dist_);

        update_cmd_vel(linear_speed_m_s_, gentle_turn_speed_rad_s_);
    }

    else
    {
        rule = 6;

        fprintf(stderr, "[RULE 6] Normal corridor -> MOVE FORWARD\n");

        update_cmd_vel(linear_speed_m_s_, 0.0);
    }

    RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        500,
        "rule=%d cmd=(%.3f,%.3f) F=%.3f FL=%.3f LF=%.3f RF=%.3f FR=%.3f",
        rule,
        g_last_cmd_v,
        g_last_cmd_w,
        scan_data_[FRONT],
        scan_data_[FRONT_LEFT],
        scan_data_[LEFT_FRONT],
        scan_data_[RIGHT_FRONT],
        scan_data_[FRONT_RIGHT]);

}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<WallFollower>());

    rclcpp::shutdown();

    return 0;
}
