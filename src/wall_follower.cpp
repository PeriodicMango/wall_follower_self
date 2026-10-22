// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Original Last Modified: 2024.10.14
//

#include "wall_follower/wall_follower.hpp"
#include <memory>
#include <cmath>
#include <cstdio>

// ===== VISION CHANGE 1 =====
// Receive marker positions published by see_marker.py.
#include <geometry_msgs/msg/point_stamped.hpp>
// ===== VISION CHANGE 1 END =====

using namespace std::chrono_literals;

static bool g_scan_received = false;
static bool g_scan_meta_printed = false;
static double g_last_cmd_v = 0.0;
static double g_last_cmd_w = 0.0;

// ===== VISION CHANGE 2 START =====
// /marker_position convention:
// point.x = forward distance from robot/camera, metres
// point.y = lateral position, +left / -right, metres
// point.z = 0
static rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr g_marker_sub;
static bool g_marker_detected = false;
static double g_marker_x = 0.0;
static double g_marker_y = 0.0;
static double g_last_marker_time_sec = -1.0;

// Vision parameters for testing.
constexpr double MARKER_TIMEOUT_SEC = 0.50;
constexpr double MARKER_AVOID_DISTANCE = 0.70;
constexpr double MARKER_TURN_SPEED = 0.40;
// ===== VISION CHANGE 2 END =====

namespace
{
    constexpr double FRONT_BLOCK_DIST = 0.5;
    constexpr double LEFT_OPEN_DIST = 0.60;
    constexpr double FRONT_LEFT_CLOSE = 0.5;
    constexpr double FRONT_RIGHT_CLOSE = 0.5;
    constexpr double LEFT_TOO_FAR_DIST = 0.50;
    constexpr double LINEAR_SPEED = 0.15;
    constexpr double TURN_LINEAR_SPEED = 0.10;
    constexpr double TURN_SPEED = 0.6;
    constexpr double GENTLE_TURN_SPEED = 0.30;
}

WallFollower::WallFollower()
: Node("wall_follower_node")
{
    for (int i = 0; i < 12; i++)
    {
        scan_data_[i] = 0.0;
    }

    robot_pose_ = 0.0;
    near_start = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel",
            qos);

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

    // ===== VISION CHANGE 3 START =====
    // Subscribe to marker positions from see_marker.py.
    g_marker_sub =
        this->create_subscription<geometry_msgs::msg::PointStamped>(
            "marker_position",
            qos,
            [this](const geometry_msgs::msg::PointStamped::SharedPtr msg)
            {
                g_marker_detected = true;
                g_marker_x = msg->point.x;
                g_marker_y = msg->point.y;
                g_last_marker_time_sec = this->now().seconds();

                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    500,
                    "[VISION] marker x=%.3f y=%.3f",
                    g_marker_x,
                    g_marker_y);
            });
    // ===== VISION CHANGE 3 END =====

    update_timer_ =
        this->create_wall_timer(
            50ms,
            std::bind(
                &WallFollower::update_callback,
                this));

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
            fprintf(
                stderr,
                "[ODOM] Robot has left the starting area\n");
        }
    }
    else if (
        std::fabs(current_x - start_x) < START_RANGE &&
        std::fabs(current_y - start_y) < START_RANGE)
    {
        fprintf(stderr, "[ODOM] Near start!!\n");
        near_start = true;
        first = true;
        start_moving = true;
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

        if (
            !std::isfinite(r) ||
            r < msg->range_min ||
            r > msg->range_max)
        {
            continue;
        }

        if (r < closest)
        {
            closest = r;
        }
    }

    for (int angle = 0; angle < BEAM_WIDTH; angle++)
    {
        const float r = msg->ranges.at(angle);

        if (
            !std::isfinite(r) ||
            r < msg->range_min ||
            r > msg->range_max)
        {
            continue;
        }

        if (r < closest)
        {
            closest = r;
        }
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

            if (
                !std::isfinite(r) ||
                r < msg->range_min ||
                r > msg->range_max)
            {
                continue;
            }

            if (r < closest)
            {
                closest = r;
            }
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
            fprintf(
                stderr,
                "[WAIT] No valid /scan received yet. Robot stays stopped.\n");
            waiting_message_printed = true;
        }

        return;
    }

    if (near_start)
    {
        update_cmd_vel(0.0, 0.0);
        fprintf(stderr, "[STOP] Robot returned near start.\n");
        exit(0);
    }

    // ===== VISION CHANGE 4 START =====
    // Clear marker detection if no new vision message arrives for 0.5 seconds.
    if (
        g_marker_detected &&
        g_last_marker_time_sec >= 0.0)
    {
        const double marker_age =
            this->now().seconds() - g_last_marker_time_sec;

        if (marker_age > MARKER_TIMEOUT_SEC)
        {
            g_marker_detected = false;
        }
    }
    // ===== VISION CHANGE 4 END =====

    int rule = 0;

    fprintf(
        stderr,
        "[DATA] FRONT=%.3f | FRONT_LEFT=%.3f | LEFT_FRONT=%.3f | FRONT_RIGHT=%.3f\n",
        scan_data_[FRONT],
        scan_data_[FRONT_LEFT],
        scan_data_[LEFT_FRONT],
        scan_data_[FRONT_RIGHT]);

    if (scan_data_[FRONT] < FRONT_BLOCK_DIST)
    {
        rule = 1;

        fprintf(
            stderr,
            "[RULE 1] FRONT < %.2f -> TURN RIGHT IN PLACE\n",
            FRONT_BLOCK_DIST);

        update_cmd_vel(0.0, -TURN_SPEED - 0.4);
    }

    // ===== VISION CHANGE 5 START =====
    // Avoid a nearby marker before continuing normal wall-following rules.
    else if (
        g_marker_detected &&
        g_marker_x > 0.0 &&
        std::hypot(g_marker_x, g_marker_y) < MARKER_AVOID_DISTANCE)
    {
        rule = 7;

        double marker_distance =
            std::hypot(g_marker_x, g_marker_y);

        if (g_marker_y >= 0.0)
        {
            fprintf(
                stderr,
                "[RULE 7] MARKER LEFT/CENTRE dist=%.3f x=%.3f y=%.3f -> TURN RIGHT\n",
                marker_distance,
                g_marker_x,
                g_marker_y);

            update_cmd_vel(
                0.0,
                -MARKER_TURN_SPEED);
        }
        else
        {
            fprintf(
                stderr,
                "[RULE 7] MARKER RIGHT dist=%.3f x=%.3f y=%.3f -> TURN LEFT\n",
                marker_distance,
                g_marker_x,
                g_marker_y);

            update_cmd_vel(
                0.0,
                MARKER_TURN_SPEED);
        }
    }
    // ===== VISION CHANGE 5 END =====

    else if (scan_data_[FRONT_LEFT] < FRONT_LEFT_CLOSE)
    {
        rule = 3;

        fprintf(
            stderr,
            "[RULE 3] FRONT_LEFT < %.2f -> TOO CLOSE LEFT, STEER RIGHT\n",
            FRONT_LEFT_CLOSE);

        update_cmd_vel(
            LINEAR_SPEED,
            -0.40);
    }
    else if (scan_data_[LEFT_FRONT] < 0.3)
    {
        fprintf(
            stderr,
            "[LEFT_FRONT CLOSE] LEFT_FRONT < 0.30 -> KEEP STEERING RIGHT\n");

        update_cmd_vel(
            TURN_LINEAR_SPEED,
            -0.40);
    }
    else if (scan_data_[LEFT_FRONT] > LEFT_OPEN_DIST)
    {
        rule = 2;

        fprintf(
            stderr,
            "[RULE 2] LEFT_FRONT > %.2f -> LEFT WALL OPEN, TURN LEFT\n",
            LEFT_OPEN_DIST);

        update_cmd_vel(
            TURN_LINEAR_SPEED,
            TURN_SPEED);
    }
    else if (scan_data_[FRONT_RIGHT] < FRONT_RIGHT_CLOSE)
    {
        rule = 4;

        fprintf(
            stderr,
            "[RULE 4] FRONT_RIGHT < %.2f -> TOO CLOSE RIGHT, STEER LEFT\n",
            FRONT_RIGHT_CLOSE);

        update_cmd_vel(
            LINEAR_SPEED,
            0.40);
    }
    else if (scan_data_[LEFT_FRONT] > LEFT_TOO_FAR_DIST)
    {
        rule = 5;

        fprintf(
            stderr,
            "[RULE 5] LEFT_FRONT > %.2f -> SLIGHTLY FAR FROM LEFT, STEER LEFT\n",
            LEFT_TOO_FAR_DIST);

        update_cmd_vel(
            LINEAR_SPEED,
            GENTLE_TURN_SPEED);
    }
    else
    {
        rule = 6;

        fprintf(
            stderr,
            "[RULE 6] Normal corridor -> MOVE FORWARD\n");

        update_cmd_vel(
            LINEAR_SPEED,
            0.0);
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