// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <cmath>
#include <memory>


using namespace std::chrono_literals;

/************************************************************
** V1 tunable parameters
** V1 only ever changes these numbers (TODO section 1.3: one experiment =
** one parameter = one bag). They carry the original handout values, so
** 1.1 is a pure refactor: behaviour is unchanged, and any later change
** seen in a bag is attributable to the single parameter that was edited.
************************************************************/
// distance thresholds [m]
#define TH_FRONT_STOP	0.7	// front closer than this -> turn in place
#define TH_LEFT_OPEN	0.9	// left-front open beyond this -> wide left arc
#define TH_FRONT_LEFT	0.6	// front-left closer than this -> veer right
#define TH_FRONT_RIGHT	0.6	// front-right closer than this -> veer left
#define TH_LEFT_FOLLOW	0.6	// left-front still open -> keep following the wall
// velocities
#define SPEED_FWD	0.3	// forward speed [m/s]
#define SPEED_ARC	0.2	// forward speed while arcing left [m/s]
#define TURN_HARD	1.5	// hard turn rate [rad/s]

/************************************************************
** V1 state
** File-scope rather than a class member: the class declaration lives in
** the header, and the header is frozen for V1.
************************************************************/
static bool scan_valid_ = false;	// set once a frame holding usable ranges has arrived

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
		scan_data_[i] = 0.0;

	robot_pose_ = 0.0;
	near_start = false;

	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

	// Initialise publishers
	cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", qos);

	// Initialise subscribers
	scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
		"scan", \
		rclcpp::SensorDataQoS(), \
		std::bind(
			&WallFollower::scan_callback, \
			this, \
			std::placeholders::_1));
	odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
		"odom", qos, std::bind(&WallFollower::odom_callback, this, std::placeholders::_1));

	/************************************************************
	** Initialise ROS timers
	************************************************************/
	update_timer_ = this->create_wall_timer(10ms, std::bind(&WallFollower::update_callback, this));

	RCLCPP_INFO(this->get_logger(), "Wall follower node has been initialised");
}

WallFollower::~WallFollower()
{
	RCLCPP_INFO(this->get_logger(), "Wall follower node has been terminated");
}

/********************************************************************************
** Callback functions for ROS subscribers
********************************************************************************/

#define START_RANGE	0.2

void WallFollower::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
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

	double current_x =  msg->pose.pose.position.x;
	double current_y =  msg->pose.pose.position.y;
	if (first)
	{
		start_x = current_x;
		start_y = current_y;
		first = false;
	}
	else if (start_moving)
	{
		if (fabs(current_x - start_x) > START_RANGE || fabs(current_y - start_y) > START_RANGE)
			start_moving = false;
	}
	else if (fabs(current_x - start_x) < START_RANGE && fabs(current_y - start_y) < START_RANGE)
	{
		RCLCPP_INFO(this->get_logger(), "Near start!!");
		near_start = true;
		first = true;
		start_moving = true;
	}
}

#define BEAM_WIDTH 15

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	uint16_t scan_angle[12] = {0, 30, 60, 90, 120, 150, 180, 210, 240, 270, 300, 330};

	/************************************************************
	** Exception 1 (V1): never mistake a dead scan for "wall ahead".
	** An all-invalid frame leaves scan_data_ at its constructor value 0.0,
	** which makes FRONT < TH_FRONT_STOP true for *every* heading: the robot
	** then spins in place for ever and never arms near_start. Only a frame
	** carrying usable ranges may set the flag, and the flag latches, so a
	** single bad frame cannot interrupt a robot that is under way.
	************************************************************/
	int valid = 0;
	for (const auto & r : msg->ranges)
		if (std::isfinite(r) && r > msg->range_min) valid++;
	if (valid == 0)
		return;
	if (!scan_valid_)
		RCLCPP_INFO(
			this->get_logger(), "scan usable: %d/%zu beams beyond range_min",
			valid, msg->ranges.size());
	scan_valid_ = true;

	double closest = msg->range_max;
	for (int angle = 360-BEAM_WIDTH; angle < 360; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	for (int angle = 0; angle < BEAM_WIDTH; angle++)
		if (msg->ranges.at(angle) < closest)
			closest = msg->ranges.at(angle);
	scan_data_[0] = closest;

	for (int i = 1; i < 12; i++)
	{
		closest = msg->range_max;
		for (int angle = scan_angle[i]-BEAM_WIDTH; angle < scan_angle[i]+BEAM_WIDTH; angle++)
			if (msg->ranges.at(angle) < closest)
				closest = msg->ranges.at(angle);
		scan_data_[i] = closest;
	}
}

void WallFollower::update_cmd_vel(double linear, double angular)
{
	geometry_msgs::msg::Twist cmd_vel;
	cmd_vel.linear.x = linear;
	cmd_vel.angular.z = angular;

	cmd_vel_pub_->publish(cmd_vel);
}

/********************************************************************************
** Update functions
********************************************************************************/

void WallFollower::update_callback()
{
	/************************************************************
	** Exception 1 (V1): no usable scan yet -> hold still, and say so.
	** Sending zero rather than a bare return matters: a bare return
	** publishes nothing, and the OpenCR firmware has no cmd_vel timeout,
	** so the robot would keep driving on the last command it was given
	** (the same mechanism as "the robot keeps turning after Ctrl-C").
	************************************************************/
	if (!scan_valid_)
	{
		RCLCPP_WARN_THROTTLE(
			this->get_logger(), *this->get_clock(), 2000,
			"no usable scan yet - holding still");
		update_cmd_vel(0.0, 0.0);
		return;
	}

	/************************************************************
	** Branch table: decide first, publish once at the end.
	** The thresholds and the two velocities are unchanged from the handout;
	** exception 2 only makes the decision visible (branch number, the five
	** beams it is decided from, and the command that follows from it).
	************************************************************/
	int branch = 0;			// sentinel: 0 is never a real branch
	double cmd_v = 0.0, cmd_w = 0.0;

	if (near_start) {update_cmd_vel(0.0, 0.0); exit(0);}
	else if (scan_data_[LEFT_FRONT] > TH_LEFT_OPEN) {branch = 1; cmd_v = SPEED_ARC; cmd_w = TURN_HARD;}
	else if (scan_data_[FRONT] < TH_FRONT_STOP) {branch = 2; cmd_v = 0.0; cmd_w = -TURN_HARD;}
	else if (scan_data_[FRONT_LEFT] < TH_FRONT_LEFT) {branch = 3; cmd_v = SPEED_FWD; cmd_w = -TURN_HARD;}
	else if (scan_data_[FRONT_RIGHT] < TH_FRONT_RIGHT) {branch = 4; cmd_v = SPEED_FWD; cmd_w = TURN_HARD;}
	else if (scan_data_[LEFT_FRONT] > TH_LEFT_FOLLOW) {branch = 5; cmd_v = SPEED_FWD; cmd_w = TURN_HARD;}
	else {branch = 6; cmd_v = SPEED_FWD; cmd_w = 0.0;}

	update_cmd_vel(cmd_v, cmd_w);

	/************************************************************
	** Exception 2 (V1): two lines a second, so a run can be diagnosed from
	** the terminal alone. 500 ms undersamples the measured branch-switch
	** rate (~1.3 per sim second) on purpose: the log shows which branches
	** occur and what the beams looked like, not an exact switch count -
	** for that, record a bag and run policy_replay.py.
	** RF (300 deg) is not a branch input; it is logged as the mirror of LF
	** so that left/right asymmetry is visible.
	************************************************************/
	RCLCPP_INFO_THROTTLE(
		this->get_logger(), *this->get_clock(), 500,
		"branch=%d cmd=(%.3f,%.3f) F=%.3f FL=%.3f LF=%.3f RF=%.3f FR=%.3f",
		branch, cmd_v, cmd_w,
		scan_data_[FRONT], scan_data_[FRONT_LEFT], scan_data_[LEFT_FRONT],
		scan_data_[RIGHT_FRONT], scan_data_[FRONT_RIGHT]);
}



/*******************************************************************************
** Main
*******************************************************************************/
int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();

	return 0;
}
