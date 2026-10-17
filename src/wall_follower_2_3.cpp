// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Original Last Modified: 2024.10.14
//

#include "wall_follower/wall_follower.hpp"

#include <memory>

#include <cmath>

#include <cstdio>

using namespace std::chrono_literals;

// /scan check parameters
static bool g_scan_received = false;

static bool g_scan_meta_printed = false;

static double g_last_cmd_v = 0.0;
static double g_last_cmd_w = 0.0;

// Modifiable parameters
namespace
{
    // Front distance check
    constexpr double FRONT_BLOCK_DIST = 0.5;

    // Left opening check
    constexpr double LEFT_OPEN_DIST = 0.60;

    // Front left obstacle check
    constexpr double FRONT_LEFT_CLOSE = 0.5;

    // Front right obstacle check
    constexpr double FRONT_RIGHT_CLOSE = 0.5;

    // Slight wall adjustment
    constexpr double LEFT_TOO_FAR_DIST = 0.50;

    // Linear speed
    constexpr double LINEAR_SPEED = 0.15;

    // Turn linear speed
    constexpr double TURN_LINEAR_SPEED = 0.10;

    // Turn angular speed
    constexpr double TURN_SPEED = 0.6;

    // Gentle turn angular speed
    constexpr double GENTLE_TURN_SPEED = 0.30;
}

// Constructor
WallFollower::WallFollower()
: Node("wall_follower_node")
{
    // Initialize LiDAR to 0.0
    for (int i = 0; i < 12; i++)
    {
        scan_data_[i] = 0.0;
    }

    robot_pose_ = 0.0;

    near_start = false;

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // publisher to cmd_vel
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

    // subscriber to /scan LiDAR
    scan_sub_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan",

            rclcpp::SensorDataQoS(),

            std::bind(
                &WallFollower::scan_callback,
                this,
                std::placeholders::_1));

    // subscriber to odom
    odom_sub_ =
        this->create_subscription<nav_msgs::msg::Odometry>(
            "odom",

            qos,

            std::bind(
                &WallFollower::odom_callback,
                this,
                std::placeholders::_1));

    // Timer for update callback
    update_timer_ =
        this->create_wall_timer(
            50ms,
            std::bind(&WallFollower::update_callback, this));

    // Print info to show success construction
    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been initialised");
}

// Destructor
WallFollower::~WallFollower()
{
    // Print info
    RCLCPP_INFO(
        this->get_logger(),
        "Wall follower node has been terminated");
}

// Odom logic
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

    // Current position
    double current_x = msg->pose.pose.position.x;
    double current_y = msg->pose.pose.position.y;

    // First odom frame: record the start point
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

    // Wait until the robot has left the start area
    else if (start_moving)
    {
        // Axial distance check against LEAVE_RANGE
        if (
            std::fabs(current_x - start_x) > LEAVE_RANGE ||
            std::fabs(current_y - start_y) > LEAVE_RANGE)
        {
            start_moving = false;

            fprintf(stderr, "[ODOM] Robot has left the starting area\n");
        }
    }

    // Back near the start point
    else if (
        std::fabs(current_x - start_x) < START_RANGE &&
        std::fabs(current_y - start_y) < START_RANGE)
    {
        fprintf(stderr, "[ODOM] Near start!!\n");

        // update_callback stops the robot and exits
        near_start = true;

        // Reset state
        first = true;
        start_moving = true;
    }
}

// /scan callback: compress a frame into 12 sectors

// Each sector: centre +/-15 deg (30 deg wide)
#define BEAM_WIDTH 15

void WallFollower::scan_callback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // Sector centres: 0, 30, ..., 330 deg
    uint16_t scan_angle[12] =
    {
        0, 30, 60, 90, 120, 150,
        180, 210, 240, 270, 300, 330
    };

    // Print LiDAR metadata once
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

    // Fewer than 360 ranges: skip this frame
    if (msg->ranges.size() < 360)
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "LaserScan has only %zu ranges; this code expects at least 360",
            msg->ranges.size());
        return;
    }
    // Sector 0 (FRONT): 345-359 and 0-14 deg

    // Start from range_max
    double closest = msg->range_max;

    // 345-359 deg
    for (int angle = 360 - BEAM_WIDTH; angle < 360; angle++)
    {
        const float r = msg->ranges.at(angle);

        // Skip invalid returns
        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;

        // Keep the closest
        if (r < closest)
            closest = r;
    }

    // 0-14 deg
    for (int angle = 0; angle < BEAM_WIDTH; angle++)
    {
        const float r = msg->ranges.at(angle);

        if (!std::isfinite(r) || r < msg->range_min || r > msg->range_max)
            continue;

        if (r < closest)
            closest = r;
    }

    // Store into scan_data_[FRONT]
    scan_data_[0] = closest;

    // Sectors 1..11: 30-330 deg
    for (int i = 1; i < 12; i++)
    {
        closest = msg->range_max;

        // +/-15 deg around the sector centre
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

    // At least one valid frame received
    g_scan_received = true;
}

// Publish to /cmd_vel
void WallFollower::update_cmd_vel(
    double linear,
    double angular)
{
    geometry_msgs::msg::Twist cmd_vel;

    // linear.x: forward speed (m/s)
    cmd_vel.linear.x = linear;

    // angular.z: turn rate (rad/s, + = left)
    cmd_vel.angular.z = angular;

    cmd_vel_pub_->publish(cmd_vel);

    // Remember the published value for the throttled log
    g_last_cmd_v = linear;
    g_last_cmd_w = angular;
}

// 20 Hz control loop
void WallFollower::update_callback()
{
    // No /scan yet: stay stopped
    if (!g_scan_received)
    {
        update_cmd_vel(0.0, 0.0);

        // Print [WAIT] once
        static bool waiting_message_printed = false;
        if (!waiting_message_printed)
        {
            fprintf(stderr, "[WAIT] No valid /scan received yet. Robot stays stopped.\n");
            waiting_message_printed = true;
        }

        return;
    }

    // Back near start: stop and exit
    if (near_start)
    {
        update_cmd_vel(0.0, 0.0);

        fprintf(stderr, "[STOP] Robot returned near start.\n");

        exit(0);
    }

    // Branch taken this cycle
    int rule = 0;

    // Four distances used by the rules
    fprintf(
        stderr,
        "[DATA] FRONT=%.3f | FRONT_LEFT=%.3f | LEFT_FRONT=%.3f | FRONT_RIGHT=%.3f\n",
        scan_data_[FRONT],
        scan_data_[FRONT_LEFT],
        scan_data_[LEFT_FRONT],
        scan_data_[FRONT_RIGHT]);

    // R1: front blocked -> turn right in place
    if (scan_data_[FRONT] < FRONT_BLOCK_DIST)
    {
        rule = 1;
        fprintf(
            stderr,
            "[RULE 1] FRONT < %.2f -> TURN RIGHT IN PLACE\n",
            FRONT_BLOCK_DIST);

        update_cmd_vel(0.0, -TURN_SPEED-0.4);
    }

    // R3: front-left too close -> steer right
    else if (scan_data_[FRONT_LEFT] < FRONT_LEFT_CLOSE)
    {
        rule = 3;
        fprintf(
            stderr,
            "[RULE 3] FRONT_LEFT < %.2f -> TOO CLOSE LEFT, STEER RIGHT\n",
            FRONT_LEFT_CLOSE);


        update_cmd_vel(LINEAR_SPEED, -0.40);       
    }

    // 60 deg guard: keep steering right while the marker slides past
    else if (scan_data_[LEFT_FRONT] < 0.3)
    {
        fprintf(
            stderr,
            "[LEFT_FRONT CLOSE] LEFT_FRONT < 0.30 -> KEEP STEERING RIGHT\n");

        update_cmd_vel(TURN_LINEAR_SPEED, -0.40);
    }

    // R2: left wall opens up -> turn left
    else if (scan_data_[LEFT_FRONT] > LEFT_OPEN_DIST)
    {
        rule = 2;
        fprintf(
            stderr,
            "[RULE 2] LEFT_FRONT > %.2f -> LEFT WALL OPEN, TURN LEFT\n",
            LEFT_OPEN_DIST);

        update_cmd_vel(TURN_LINEAR_SPEED, TURN_SPEED);
    }

    // R4: front-right too close -> steer left
    else if (scan_data_[FRONT_RIGHT] < FRONT_RIGHT_CLOSE)
    {
        rule = 4;
        fprintf(
            stderr,
            "[RULE 4] FRONT_RIGHT < %.2f -> TOO CLOSE RIGHT, STEER LEFT\n",
            FRONT_RIGHT_CLOSE);

        update_cmd_vel(LINEAR_SPEED, 0.40);
    }

    // R5: left wall slightly far -> gentle left correction
    else if (scan_data_[LEFT_FRONT] > LEFT_TOO_FAR_DIST)
    {
        rule = 5;
        fprintf(
            stderr,
            "[RULE 5] LEFT_FRONT > %.2f -> SLIGHTLY FAR FROM LEFT, STEER LEFT\n",
            LEFT_TOO_FAR_DIST);

        update_cmd_vel(LINEAR_SPEED, GENTLE_TURN_SPEED);
    }

    // R6: nothing triggered -> straight
    else
    {
        rule = 6;

        fprintf(stderr, "[RULE 6] Normal corridor -> MOVE FORWARD\n");

        update_cmd_vel(LINEAR_SPEED, 0.0);
    }

    // Throttled 0.5 s log: branch, cmd and five sectors
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

// main
int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    // Spin the node until shutdown
    rclcpp::spin(std::make_shared<WallFollower>());

    rclcpp::shutdown();

    return 0;
}
