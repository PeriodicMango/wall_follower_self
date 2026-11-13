"""Localize and navigate on .103 using its existing Pi-side Twist guard."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    params = str(Path(__file__).resolve().parents[1] / "config" / "nav2_robot103.yaml")
    map_yaml = ParameterValue(LaunchConfiguration("map"), value_type=str)
    raw_velocity = "/turtlebot_test/nav_cmd_vel"
    guard_input = "/turtlebot_test/wall_cmd_vel"

    def server(package, name, *, remappings=(), overrides=None):
        parameters = [params]
        if overrides:
            parameters.append(overrides)
        return Node(
            package=package,
            executable=name,
            name=name,
            parameters=parameters,
            remappings=list(remappings),
            output="screen",
        )

    def lifecycle_manager(group, names):
        return Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name=f"lifecycle_manager_{group}",
            output="screen",
            parameters=[{"node_names": names, "autostart": True, "use_sim_time": False}],
        )

    # Both motion producers use the same smoother input. The only allowed
    # publisher of the actual base topic /cmd_vel is the guard on the Pi.
    to_smoother = [("cmd_vel", raw_velocity)]
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Absolute YAML path of the physical maze map"),
        server("nav2_map_server", "map_server", overrides={"yaml_filename": map_yaml}),
        server("nav2_amcl", "amcl"),
        lifecycle_manager("localization", ["map_server", "amcl"]),
        server("nav2_controller", "controller_server", remappings=to_smoother),
        server("nav2_smoother", "smoother_server"),
        server("nav2_planner", "planner_server"),
        server("nav2_behaviors", "behavior_server", remappings=to_smoother),
        server("nav2_bt_navigator", "bt_navigator"),
        server("nav2_waypoint_follower", "waypoint_follower"),
        server(
            "nav2_velocity_smoother",
            "velocity_smoother",
            remappings=[("cmd_vel", raw_velocity), ("cmd_vel_smoothed", guard_input)],
        ),
        lifecycle_manager("navigation", [
            "controller_server", "smoother_server", "planner_server", "behavior_server",
            "bt_navigator", "waypoint_follower", "velocity_smoother",
        ]),
    ])
