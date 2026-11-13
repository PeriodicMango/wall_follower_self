#!/usr/bin/env bash
set -eu
feature=${1:-nav}
if [[ $# -gt 1 || ( "$feature" != nav && "$feature" != mapping ) ]]; then echo 'Usage: ./ubuntu/run.sh check [nav|mapping]' >&2; exit 2; fi
for binary in ros2 rviz2 ssh ip; do command -v "$binary" >/dev/null || { echo "Missing executable: $binary" >&2; exit 1; }; done
packages=(rmw_cyclonedds_cpp nav2_map_server tf2_geometry_msgs)
if [[ "$feature" == nav ]]; then
 packages+=(nav2_simple_commander nav2_common nav2_bringup nav2_amcl nav2_controller nav2_planner nav2_behaviors nav2_bt_navigator nav2_velocity_smoother nav2_lifecycle_manager nav2_smoother nav2_waypoint_follower rclcpp_components)
else
 command -v cmake >/dev/null
 packages+=(cartographer_ros cv_bridge ament_cmake rclcpp sensor_msgs nav_msgs geometry_msgs std_msgs tf2)
fi
for pkg in "${packages[@]}"; do ros2 pkg prefix "$pkg" >/dev/null || { echo "Missing ROS package: $pkg" >&2; exit 1; }; done
/usr/bin/python3 - "$feature" <<'PY'
import sys
import rclpy, tf2_geometry_msgs, yaml, numpy, PIL.Image
if sys.argv[1] == 'nav':
    from nav2_simple_commander.robot_navigator import BasicNavigator
else:
    import cv2, cv_bridge
PY
echo "Local Ubuntu Humble $feature dependencies checked (no robot or motor connection)."
