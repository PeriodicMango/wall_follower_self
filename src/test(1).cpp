// Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
// Author: Claude Sammut
// Last Modified: 2024.10.14

// Use this code as the basis for a wall follower

#include "wall_follower/wall_follower.hpp"

#include <memory>

using namespace std::chrono_literals;


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

		fprintf(
			stderr,
			"Start position: x=%.3f y=%.3f\n",
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

			fprintf(
				stderr,
				"Robot has left the starting area\n"
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
		fprintf(
			stderr,
			"Near start!!\n"
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
			msg->ranges.at(angle) > 0.1 &&
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
			msg->ranges.at(angle) > 0.1 &&
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
				msg->ranges.at(angle) > 0.1 &&
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
	if (near_start)
	{
		fprintf(
			stderr,
			"[STOP] Robot returned near start\n"
		);

		update_cmd_vel(
			0.0,
			0.0
		);

		exit(0);
	}


	/************************************************************
	** Print useful LiDAR values
	************************************************************/
	fprintf(
		stderr,
		"FRONT: %.3f | FRONT_LEFT: %.3f | LEFT_FRONT: %.3f | FRONT_RIGHT: %.3f\n",
		scan_data_[FRONT],
		scan_data_[FRONT_LEFT],
		scan_data_[LEFT_FRONT],
		scan_data_[FRONT_RIGHT]
	);


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
		fprintf(
			stderr,
			"[1] Left wall disappeared -> TURN LEFT\n"
		);

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
		fprintf(
			stderr,
			"[2] Wall ahead -> TURN RIGHT\n"
		);

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
		fprintf(
			stderr,
			"[3] Too close to LEFT wall -> STEER RIGHT\n"
		);

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
		fprintf(
			stderr,
			"[4] Too close to RIGHT wall -> STEER LEFT\n"
		);

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
		fprintf(
			stderr,
			"[5] Too far from LEFT wall -> STEER LEFT\n"
		);

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
		fprintf(
			stderr,
			"[6] Move forward\n"
		);

		update_cmd_vel(
			0.15,
			0.0
		);
	}
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