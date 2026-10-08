// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <cmath>
#include <memory>

using namespace std::chrono_literals;


/************************************************************
** Last command actually published. Kept only so that the throttled log line
** can report what was sent on this tick.
************************************************************/
static double last_cmd_v = 0.0;
static double last_cmd_w = 0.0;


/************************************************************
** scan_valid_: set once a frame carrying usable ranges has arrived.
**
** Without it an all-invalid frame (dead LDS, ranges never arriving) leaves
** scan_data_ at its constructor value 0.0, which makes FRONT < 0.40 true for
** *every* heading: rule 2 then spins the robot in place for ever and never
** arms near_start. File scope rather than a class member because the class
** lives in the header.
************************************************************/
static bool scan_valid_ = false;


/********************************************************************************
** Constructor
********************************************************************************/

WallFollower::WallFollower()
: Node("wall_follower_node")
{
	/************************************************************
	** Initialise variables
	************************************************************/
	for (int i = 0; i < 12; i++)
	{
		scan_data_[i] = 0.0;
	}

	robot_pose_ = 0.0;
	near_start = false;


	/************************************************************
	** Initialise ROS publishers and subscribers
	************************************************************/
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10));


	// Initialise publisher
	cmd_vel_pub_ =
		this->create_publisher<geometry_msgs::msg::Twist>(
			"cmd_vel",
			qos
		);


	// Initialise LaserScan subscriber
	scan_sub_ =
		this->create_subscription<sensor_msgs::msg::LaserScan>(
			"scan",
			rclcpp::SensorDataQoS(),
			std::bind(
				&WallFollower::scan_callback,
				this,
				std::placeholders::_1
			)
		);


	// Initialise odometry subscriber
	odom_sub_ =
		this->create_subscription<nav_msgs::msg::Odometry>(
			"odom",
			qos,
			std::bind(
				&WallFollower::odom_callback,
				this,
				std::placeholders::_1
			)
		);


	/************************************************************
	** Initialise ROS timer
	**
	** 50 ms = 20 Hz
	************************************************************/
	update_timer_ =
		this->create_wall_timer(
			50ms,
			std::bind(
				&WallFollower::update_callback,
				this
			)
		);


	RCLCPP_INFO(
		this->get_logger(),
		"Wall follower node has been initialised"
	);
}


/********************************************************************************
** Destructor
********************************************************************************/

WallFollower::~WallFollower()
{
	RCLCPP_INFO(
		this->get_logger(),
		"Wall follower node has been terminated"
	);
}


/********************************************************************************
** Odometry callback
********************************************************************************/

#define START_RANGE 0.2
#define LEAVE_RANGE 1.0

void WallFollower::odom_callback(
	const nav_msgs::msg::Odometry::SharedPtr msg)
{
	static bool first = true;
	static bool start_moving = true;


	/************************************************************
	** Get robot orientation
	************************************************************/
	tf2::Quaternion q(
		msg->pose.pose.orientation.x,
		msg->pose.pose.orientation.y,
		msg->pose.pose.orientation.z,
		msg->pose.pose.orientation.w
	);

	tf2::Matrix3x3 m(q);

	double roll, pitch, yaw;

	m.getRPY(
		roll,
		pitch,
		yaw
	);

	robot_pose_ = yaw;


	/************************************************************
	** Get robot position
	************************************************************/
	double current_x =
		msg->pose.pose.position.x;

	double current_y =
		msg->pose.pose.position.y;


	/************************************************************
	** Record starting position
	************************************************************/
	if (first)
	{
		start_x = current_x;
		start_y = current_y;

		first = false;

		RCLCPP_INFO(
			this->get_logger(),
			"start position: x=%.3f y=%.3f",
			start_x,
			start_y
		);
	}


	/************************************************************
	** Robot must leave the starting area first
	************************************************************/
	else if (start_moving)
	{
		if (
			fabs(current_x - start_x) > LEAVE_RANGE ||
			fabs(current_y - start_y) > LEAVE_RANGE
		)
		{
			start_moving = false;

			RCLCPP_INFO(
				this->get_logger(),
				"robot has left the starting area"
			);
		}
	}


	/************************************************************
	** Detect return near starting position
	************************************************************/
	else if (
		fabs(current_x - start_x) < START_RANGE &&
		fabs(current_y - start_y) < START_RANGE
	)
	{
		RCLCPP_INFO(
			this->get_logger(),
			"Near start!!"
		);

		near_start = true;

		first = true;
		start_moving = true;
	}
}


/********************************************************************************
** Laser scan callback
********************************************************************************/

#define BEAM_WIDTH 15

void WallFollower::scan_callback(
	const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
	/************************************************************
	** Guard: never treat a dead scan as real geometry.
	**
	** Only a frame carrying at least one finite range beyond range_min may
	** set scan_valid_, and the flag latches, so that a single bad frame
	** cannot interrupt a robot that is under way.
	************************************************************/
	int valid = 0;
	int below_min = 0;
	int nonfinite = 0;

	double raw_min = 0.0;
	double raw_max = 0.0;
	bool have_raw = false;

	for (
		const auto & r :
		msg->ranges
	)
	{
		if (!std::isfinite(r))
		{
			nonfinite++;

			continue;
		}

		if (!have_raw)
		{
			raw_min = r;
			raw_max = r;
			have_raw = true;
		}
		else if (r < raw_min)
		{
			raw_min = r;
		}
		else if (r > raw_max)
		{
			raw_max = r;
		}

		if (r > msg->range_min)
		{
			valid++;
		}
		else
		{
			below_min++;
		}
	}


	/************************************************************
	** Scan-quality log: what the frame actually contains.
	**
	** This is the line that separates "nothing is arriving" from "frames
	** arrive but every range is unusable": below_min counts the 0.0 /
	** too-close samples and raw=[min,max] shows the actual value spread.
	************************************************************/
	RCLCPP_INFO_THROTTLE(
		this->get_logger(),
		*this->get_clock(),
		2000,
		"scan quality: n=%zu valid=%d below_min=%d nonfinite=%d raw=[%.3f,%.3f] r[0]=%.3f range=[%.3f,%.3f]",
		msg->ranges.size(),
		valid,
		below_min,
		nonfinite,
		have_raw ? raw_min : -1.0,
		have_raw ? raw_max : -1.0,
		msg->ranges.empty() ? -1.0 : msg->ranges[0],
		msg->range_min,
		msg->range_max
	);


	if (valid == 0)
	{
		return;
	}


	if (!scan_valid_)
	{
		RCLCPP_INFO(
			this->get_logger(),
			"scan usable: %d/%zu beams beyond range_min",
			valid,
			msg->ranges.size()
		);
	}


	scan_valid_ = true;


	uint16_t scan_angle[12] =
	{
		0,
		30,
		60,
		90,
		120,
		150,
		180,
		210,
		240,
		270,
		300,
		330
	};


	/************************************************************
	** FRONT sector
	**
	** FRONT is around 0 degrees:
	** 345-359 degrees + 0-14 degrees
	************************************************************/
	double closest =
		msg->range_max;


	for (
		int angle = 360 - BEAM_WIDTH;
		angle < 360;
		angle++
	)
	{
		if (
			msg->ranges.at(angle) > msg->range_min &&
			msg->ranges.at(angle) < closest
		)
		{
			closest =
				msg->ranges.at(angle);
		}
	}


	for (
		int angle = 0;
		angle < BEAM_WIDTH;
		angle++
	)
	{
		if (
			msg->ranges.at(angle) > msg->range_min &&
			msg->ranges.at(angle) < closest
		)
		{
			closest =
				msg->ranges.at(angle);
		}
	}


	scan_data_[0] =
		closest;


	/************************************************************
	** Remaining 11 sectors
	************************************************************/
	for (int i = 1; i < 12; i++)
	{
		closest =
			msg->range_max;


		for (
			int angle =
				scan_angle[i] - BEAM_WIDTH;

			angle <
				scan_angle[i] + BEAM_WIDTH;

			angle++
		)
		{
			if (
				msg->ranges.at(angle) > msg->range_min &&
				msg->ranges.at(angle) < closest
			)
			{
				closest =
					msg->ranges.at(angle);
			}
		}


		scan_data_[i] =
			closest;
	}
}


/********************************************************************************
** Publish velocity
********************************************************************************/

void WallFollower::update_cmd_vel(
	double linear,
	double angular)
{
	geometry_msgs::msg::Twist cmd_vel;

	cmd_vel.linear.x =
		linear;

	cmd_vel.angular.z =
		angular;


	cmd_vel_pub_->publish(
		cmd_vel
	);


	last_cmd_v =
		linear;

	last_cmd_w =
		angular;
}


/********************************************************************************
** Update functions
********************************************************************************/

bool pl_near;


/********************************************************************************
** Wall follower logic
********************************************************************************/

void WallFollower::update_callback()
{
	/************************************************************
	** Stop when robot returns near the starting position
	************************************************************/
	/************************************************************
	** Guard (v1 exception 1): no usable scan yet -> hold still.
	**
	** Sending zero rather than a bare return matters: a bare return publishes
	** nothing, and the OpenCR firmware has no cmd_vel timeout, so the robot
	** would keep driving on the last command it was given. Because the flag
	** latches, this only fires while no usable frame has ever arrived.
	************************************************************/
	if (!scan_valid_)
	{
		RCLCPP_WARN_THROTTLE(
			this->get_logger(),
			*this->get_clock(),
			2000,
			"no usable scan yet - holding still"
		);

		update_cmd_vel(
			0.0,
			0.0
		);

		return;
	}


	int branch = 0;			// sentinel: 0 is never a real branch


	if (near_start)
	{
		RCLCPP_INFO(
			this->get_logger(),
			"near_start: robot is back at the starting area - stopping"
		);

		update_cmd_vel(
			0.0,
			0.0
		);

		exit(0);
	}


	/************************************************************
	** Rule 1
	**
	** Left wall disappeared.
	**
	** Turn left to keep following the left wall.
	************************************************************/
	if (
		scan_data_[LEFT_FRONT] > 0.60
	)
	{
		branch = 1;

		update_cmd_vel(
			0.13,
			0.60
		);
	}


	/************************************************************
	** Rule 2
	**
	** Wall directly ahead.
	**
	** Stop moving forward and turn right.
	************************************************************/
	else if (
		scan_data_[FRONT] < 0.40
	)
	{
		branch = 2;

		update_cmd_vel(
			0.0,
			-0.60
		);
	}


	/************************************************************
	** Rule 3
	**
	** Too close to the left wall.
	**
	** Move forward while steering right.
	************************************************************/
	else if (
		scan_data_[FRONT_LEFT] < 0.25
	)
	{
		branch = 3;

		update_cmd_vel(
			0.15,
			-0.40
		);
	}


	/************************************************************
	** Rule 4
	**
	** Too close to the right wall.
	**
	** Move forward while steering left.
	************************************************************/
	else if (
		scan_data_[FRONT_RIGHT] < 0.20
	)
	{
		branch = 4;

		update_cmd_vel(
			0.15,
			0.40
		);
	}


	/************************************************************
	** Rule 5
	**
	** Slightly too far from the left wall.
	**
	** Gently steer left.
	**
	** Note:
	** Rule 1 catches LEFT_FRONT > 0.60.
	**
	** Therefore this effectively handles:
	** 0.50 < LEFT_FRONT <= 0.60
	************************************************************/
	else if (
		scan_data_[LEFT_FRONT] > 0.50
	)
	{
		branch = 5;

		update_cmd_vel(
			0.15,
			0.30
		);
	}


	/************************************************************
	** Rule 6
	**
	** Everything looks normal.
	**
	** Move straight forward.
	************************************************************/
	else
	{
		branch = 6;

		update_cmd_vel(
			0.15,
			0.0
		);
	}


	/************************************************************
	** Throttled log: branch, command, five beams
	**
	** F / FL / LF / FR are the rule inputs. RF (300 deg) is not used by
	** any rule - it is logged as the mirror of LF so that left/right
	** asymmetry is visible.
	**
	** 2 lines/s. The control loop runs at 20 Hz, so this is a deliberate
	** undersample: it shows which branches occur and what the beams looked
	** like, not an exact switch count.
	************************************************************/
	RCLCPP_INFO_THROTTLE(
		this->get_logger(),
		*this->get_clock(),
		500,
		"branch=%d cmd=(%.3f,%.3f) F=%.3f FL=%.3f LF=%.3f RF=%.3f FR=%.3f",
		branch,
		last_cmd_v,
		last_cmd_w,
		scan_data_[FRONT],
		scan_data_[FRONT_LEFT],
		scan_data_[LEFT_FRONT],
		scan_data_[RIGHT_FRONT],
		scan_data_[FRONT_RIGHT]
	);
}


/*******************************************************************************
** Main
*******************************************************************************/

int main(
	int argc,
	char ** argv)
{
	rclcpp::init(
		argc,
		argv
	);


	rclcpp::spin(
		std::make_shared<WallFollower>()
	);


	rclcpp::shutdown();


	return 0;
}