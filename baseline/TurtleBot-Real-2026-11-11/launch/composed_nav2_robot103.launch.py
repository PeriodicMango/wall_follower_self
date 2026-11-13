"""Run the guarded physical Nav2 stack in one multithreaded component container."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    source = Path(__file__).resolve().parents[1] / "config" / "nav2_robot103.yaml"
    params = ParameterFile(RewrittenYaml(
        source_file=str(source),
        param_rewrites={"map_server.ros__parameters.yaml_filename": LaunchConfiguration("map")},
    ))
    raw_velocity = "/turtlebot_test/nav_cmd_vel"
    guard_input = "/turtlebot_test/wall_cmd_vel"
    to_smoother = [("cmd_vel", raw_velocity)]

    def server(package, plugin, name, remappings=()):
        return ComposableNode(package=package, plugin=plugin, name=name,
                              parameters=[params], remappings=list(remappings))

    def lifecycle_manager(group, names):
        return ComposableNode(
            package="nav2_lifecycle_manager",
            plugin="nav2_lifecycle_manager::LifecycleManager",
            name=f"lifecycle_manager_{group}",
            parameters=[{"node_names": names, "autostart": False, "use_sim_time": False}],
        )

    nodes = [
        server("nav2_map_server", "nav2_map_server::MapServer", "map_server"),
        server("nav2_amcl", "nav2_amcl::AmclNode", "amcl"),
        server("nav2_controller", "nav2_controller::ControllerServer", "controller_server", to_smoother),
        server("nav2_smoother", "nav2_smoother::SmootherServer", "smoother_server"),
        server("nav2_planner", "nav2_planner::PlannerServer", "planner_server"),
        server("nav2_behaviors", "behavior_server::BehaviorServer", "behavior_server", to_smoother),
        server("nav2_bt_navigator", "nav2_bt_navigator::BtNavigator", "bt_navigator"),
        server("nav2_waypoint_follower", "nav2_waypoint_follower::WaypointFollower", "waypoint_follower"),
        server("nav2_velocity_smoother", "nav2_velocity_smoother::VelocitySmoother", "velocity_smoother",
               [("cmd_vel", raw_velocity), ("cmd_vel_smoothed", guard_input)]),
        lifecycle_manager("localization", ["map_server", "amcl"]),
        lifecycle_manager("navigation", [
            "controller_server", "smoother_server", "planner_server", "behavior_server",
            "bt_navigator", "waypoint_follower", "velocity_smoother",
        ]),
    ]
    return LaunchDescription([
        DeclareLaunchArgument("map", description="Absolute YAML path of the physical maze map"),
        ComposableNodeContainer(
            package="rclcpp_components",
            executable="component_container_mt",
            name="robot103_nav2_container",
            namespace="/",
            # The embedded costmap nodes read their own sections from this file.
            parameters=[params],
            composable_node_descriptions=nodes,
            output="screen",
        ),
    ])
