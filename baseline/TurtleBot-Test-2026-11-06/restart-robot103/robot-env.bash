# Source in a bash shell on the verified 192.168.1.103 robot.
source /opt/ros/humble/setup.bash
source /home/pi/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=3
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=waffle_pi
export LDS_MODEL=LDS-01
export CYCLONEDDS_URI=file:///home/pi/turtlebot_test_runs/robot103_20261106/cyclonedds-robot.xml
