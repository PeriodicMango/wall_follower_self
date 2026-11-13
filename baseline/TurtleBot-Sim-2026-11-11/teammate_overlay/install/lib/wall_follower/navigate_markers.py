#!/usr/bin/env python3
"""Visit four mapped landmarks in file order using Nav2, stopping 0.5 m away."""

import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

APPROACH_DISTANCE = 0.5
CANDIDATES = 16
# TurtleBot3's Nav2 params run NavFn with tolerance: 0.5, so ComputePathToPose
# reports success for a path that stops short of an unreachable goal. Candidates
# whose path ends further than this from the request are rejected.
GOAL_REACHED_EPS = 0.10


def read_poses(marker_file):
    with marker_file.open(newline='') as stream:
        rows = [row for row in csv.reader(stream) if row]
    if len(rows) != 5 or any(len(row) != 3 for row in rows):
        raise ValueError('Expected one odom start row followed by exactly four x,y,type rows')
    start_odom = tuple(float(value) for value in rows[0])
    markers = [(float(row[0]), float(row[1]), row[2].strip()) for row in rows[1:]]
    if not all(math.isfinite(n) for n in start_odom) or any(
        not math.isfinite(x) or not math.isfinite(y) or not name for x, y, name in markers
    ):
        raise ValueError('Marker file has non-finite coordinates or an empty marker type')
    # The file above is the only one the assignment defines, and its start row is
    # in odom. point_transformer writes a map-frame start pose beside it when it
    # could resolve map -> odom; without that file, assume the two frames share
    # an origin, which holds when Cartographer and odometry were started as one.
    sidecar = marker_file.with_suffix('.start_map.txt')
    if not sidecar.exists():
        return start_odom, start_odom, f'{marker_file.name} start row, read as map', markers
    with sidecar.open(newline='') as stream:
        map_rows = [row for row in csv.reader(stream) if row]
    if len(map_rows) != 1 or len(map_rows[0]) != 3:
        raise ValueError(f'Expected one map start pose in {sidecar}')
    start_map = tuple(float(value) for value in map_rows[0])
    if not all(math.isfinite(n) for n in start_map):
        raise ValueError('Map start pose has non-finite coordinates')
    return start_odom, start_map, sidecar.name, markers


def make_pose(nav, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2)
    pose.pose.orientation.w = math.cos(yaw / 2)
    return pose


def approach(nav, current, x, y):
    """Choose a reachable location 0.5 m from the marker with the shortest Nav2 path."""
    best = None
    for i in range(CANDIDATES):
        angle = 2 * math.pi * i / CANDIDATES
        gx = x + APPROACH_DISTANCE * math.cos(angle)
        gy = y + APPROACH_DISTANCE * math.sin(angle)
        goal = make_pose(nav, gx, gy, math.atan2(y - gy, x - gx))
        path = nav.getPath(current, goal)
        if path is None or not path.poses:
            continue
        end = path.poses[-1].pose.position
        if math.hypot(end.x - gx, end.y - gy) > GOAL_REACHED_EPS:
            continue  # Planner truncated at an obstacle; this point is unreachable.
        points = [p.pose.position for p in path.poses]
        length = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))
        if best is None or length < best[0]:
            best = (length, goal)
    return None if best is None else best[1]


def main(args=None):
    rclpy.init(args=args)
    nav = BasicNavigator('navigate_markers')
    try:
        marker_file = Path(nav.declare_parameter('marker_file', '~/markers.txt').value).expanduser()
        start_odom, start_map, start_map_source, markers = read_poses(marker_file)
        nav.get_logger().info(
            f'Original odom start: {start_odom}; '
            f'AMCL map start from {start_map_source}: {start_map}')
        current = make_pose(nav, *start_map)
        nav.setInitialPose(current)
        nav.waitUntilNav2Active()
        for index, (x, y, kind) in enumerate(markers, 1):
            goal = approach(nav, current, x, y)
            if goal is None:
                raise RuntimeError(f'Marker {index} ({kind}): no path to any 0.5 m approach point')
            nav.get_logger().info(
                f'Marker {index}/4 ({kind}) at ({x:.2f}, {y:.2f}); '
                f'navigating to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')
            if not nav.goToPose(goal):
                raise RuntimeError(f'Marker {index} ({kind}): goal rejected')
            while not nav.isTaskComplete():
                pass
            if nav.getResult() != TaskResult.SUCCEEDED:
                raise RuntimeError(f'Marker {index} ({kind}): navigation failed; stopping')
            current = goal
        nav.get_logger().info('Visited all four markers in file order')
    except (OSError, ValueError, RuntimeError) as exc:
        nav.get_logger().error(str(exc))
    except KeyboardInterrupt:
        nav.cancelTask()
    finally:
        nav.destroyNode()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
