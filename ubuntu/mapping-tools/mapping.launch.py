"""Start the two original Cartographer nodes in real time, with the dated Lua file."""
from pathlib import Path

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package = "cartographer_ros"
    libexec = Path(get_package_prefix(package)) / "lib" / package
    grid_binary = next(
        (name for name in ("cartographer_occupancy_grid_node", "occupancy_grid_node")
         if (libexec / name).is_file()),
        None,
    )
    if grid_binary is None:
        raise RuntimeError(f"Cartographer occupancy grid executable missing in {libexec}")
    return LaunchDescription([
        Node(package=package, executable="cartographer_node", name="cartographer_node",
             output="screen", parameters=[{"use_sim_time": False}],
             arguments=["-configuration_directory", str(Path(__file__).resolve().parent),
                        "-configuration_basename", "turtlebot3_lds_2d.lua"]),
        Node(package=package, executable=grid_binary, name="cartographer_occupancy_grid_node",
             output="screen", parameters=[{"use_sim_time": False}],
             arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"]),
    ])
