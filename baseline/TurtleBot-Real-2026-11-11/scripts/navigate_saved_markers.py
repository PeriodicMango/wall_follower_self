"""Visit recorded hardware landmarks after RViz localization or a verified handoff."""

import argparse
from itertools import permutations
import json
import math
import os
from pathlib import Path
import random
import secrets
import signal
import sys
import tempfile
import time

from placement_gate import PlacementChanged, wait_for_countdown
from real_run_catalog import COLORS, csv_rows, finite

REPO = Path(__file__).resolve().parents[2] / "TurtleBot-Sim-2026-11-11/teammate_repo"
sys.path.insert(0, str(REPO / "scripts"))
REAL_APPROACH_DISTANCE_M = 0.40


def read_markers(path):
    rows = csv_rows(path)
    if not 2 <= len(rows) <= 5 or not finite(rows[0], 3):
        raise ValueError(f"{path} 需要一行里程计起点和 1–4 行已保存地标。")
    seen = set()
    markers = []
    for number, row in enumerate(rows[1:], 2):
        if len(row) != 3 or not finite(row[:2], 2):
            raise ValueError(f"{path} 第 {number} 行地标坐标无效。")
        kind = row[2].strip()
        if kind not in COLORS or kind in seen:
            raise ValueError(f"{path} 第 {number} 行色序无效或重复。")
        seen.add(kind)
        markers.append((float(row[0]), float(row[1]), kind))
    return markers


def read_map_pose(path):
    data = json.loads(path.read_text())
    values = data.get("map_pose") if isinstance(data, dict) else None
    if not finite(values, 3):
        raise ValueError(f"{path} 缺少有效的已定位地图位姿 map_pose [x, y, yaw]。")
    return tuple(map(float, values))


def read_home(marker_file, home_file):
    expected = marker_file.with_suffix(".start_map.txt")
    if home_file != expected or marker_file.parent.name == "navigation_start":
        raise ValueError(f"教师模式需要原始建图地标文件旁边的地图起点 {expected}。")
    rows = csv_rows(home_file)
    if len(rows) != 1 or not finite(rows[0], 3):
        raise ValueError(f"{home_file} 需要一行地图坐标 x、y、yaw。")
    return tuple(map(float, rows[0]))


def choose_order(markers, shuffle=False, seed=None):
    if not shuffle:
        return list(markers), None
    if seed is None:
        seed = secrets.randbits(63)
    if len(markers) == 1:
        return list(markers), seed
    original = tuple(markers)
    alternatives = [order for order in permutations(markers) if order != original]
    return list(random.Random(seed).choice(alternatives)), seed


def save_order(path, record):
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile("w", dir=path.parent, delete=False) as stream:
        temp = Path(stream.name)
        try:
            json.dump(record, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.replace(temp, path)
        finally:
            temp.unlink(missing_ok=True)


def navigator():
    from nav2_simple_commander.robot_navigator import BasicNavigator

    class SavedMarkerNavigator(BasicNavigator):
        def getPath(self, start, goal, planner_id="", use_start=False):
            # Humble otherwise ignores the supplied start. Later legs use live TF.
            return super().getPath(start, goal, planner_id, use_start or start is self.captured_start)

    return SavedMarkerNavigator("navigate_saved_markers")


def approach(nav, current, x, y, distance):
    """Select the shortest feasible path to a point on the hardware approach ring."""
    from navigate_markers import CANDIDATES, GOAL_REACHED_EPS, make_pose

    best_goal, shortest = None, math.inf
    for index in range(CANDIDATES):
        angle = math.tau * index / CANDIDATES
        gx, gy = x + distance * math.cos(angle), y + distance * math.sin(angle)
        goal = make_pose(nav, gx, gy, math.atan2(y - gy, x - gx))
        path = nav.getPath(current, goal)
        if path is None or not path.poses:
            continue
        points = [p.pose.position for p in path.poses]
        if math.hypot(points[-1].x - gx, points[-1].y - gy) > GOAL_REACHED_EPS:
            continue
        length = sum(math.hypot(b.x - a.x, b.y - a.y) for a, b in zip(points, points[1:]))
        if length < shortest:
            best_goal, shortest = goal, length
    return best_goal


def last_marker_distance(nav, x, y, sent_ns):
    feedback = nav.getFeedback()
    if feedback is None:
        return None
    pose = feedback.current_pose
    stamp = pose.header.stamp.sec * 1_000_000_000 + pose.header.stamp.nanosec
    age_ns = nav.get_clock().now().nanoseconds - stamp
    if pose.header.frame_id.lstrip("/") != "map" or stamp < sent_ns or not 0 <= age_ns <= 1_000_000_000:
        return None
    distance = math.hypot(pose.pose.position.x - x, pose.pose.position.y - y)
    return distance if math.isfinite(distance) else None


def check_departure(x, y, home, tolerance, source):
    delta = math.hypot(x - home[0], y - home[1])
    if not math.isfinite(delta) or delta > tolerance:
        raise ValueError(f"{source}距原建图起点 {delta:.2f} m，要求在 {tolerance:.2f} m 内。"
                         "请把机器人放回原建图起点并重新定位。")
    return delta


def current_map_xy(nav):
    import rclpy
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    buffer = Buffer()
    listener = TransformListener(buffer, nav)
    try:
        end = time.monotonic() + 5
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(nav, timeout_sec=.1)
            try:
                transform = buffer.lookup_transform("map", "base_footprint", Time())
            except TransformException:
                continue
            stamp = Time.from_msg(transform.header.stamp)
            age = (nav.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            if abs(age) <= 2.:
                p = transform.transform.translation
                return p.x, p.y
        raise RuntimeError("未取得新鲜的 map→base_footprint 定位，无法确认机器人从建图起点出发。")
    finally:
        listener.unregister()


def wait_for_success(nav, label):
    from nav2_simple_commander.robot_navigator import TaskResult
    while not nav.isTaskComplete():
        pass  # Humble spins the action result for up to 0.1 s per check.
    result = nav.getResult()
    if result != TaskResult.SUCCEEDED:
        raise RuntimeError(f"{label}: {result.name}")


def visit(nav, markers, start, shuffle=False, *, initialize_from_handoff=False,
          distance=REAL_APPROACH_DISTANCE_M, home=None, start_tolerance=.25, before_first_goal=None, manual_start=False):
    from navigate_markers import make_pose

    if initialize_from_handoff:
        current = make_pose(nav, *start)
        nav.setInitialPose(current)
        nav.waitUntilNav2Active()
        nav.captured_start = None  # Use the live localized start for planning.
    else:
        nav.waitUntilNav2Active(localizer="map_server")
        current = nav.captured_start = make_pose(nav, *start)
    if home is not None and manual_start:
        current_map_xy(nav)
        nav.get_logger().info("已确认手动定位和 map 坐标系。返程目标使用原始建图起点。")
    elif home is not None:
        check_departure(start[0], start[1], home, start_tolerance, "本轮定位")
        delta = check_departure(*current_map_xy(nav), home, start_tolerance, "当前机器人位置")
        nav.get_logger().info(f"[START_CONFIRMED] Live map pose is {delta:.2f} m from the original mapping start")
    total = len(markers)
    for index, (x, y, kind) in enumerate(markers, 1):
        goal = approach(nav, current, x, y, distance)
        if goal is None:
            raise RuntimeError(f"Marker {index}/{total} ({kind}): no reachable {distance:.2f} m approach point")
        if index == 1 and before_first_goal is not None:
            before_first_goal(nav)
        sent_ns = nav.get_clock().now().nanoseconds
        if not nav.goToPose(goal):
            raise RuntimeError(f"Marker {index}/{total} ({kind}): goal rejected")
        nav.get_logger().info(
            f"Marker {index}/{total} ({kind}) at ({x:.2f}, {y:.2f}); "
            f"navigating to ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}), "
            f"{math.hypot(goal.pose.position.x - x, goal.pose.position.y - y):.2f} m from recorded marker"
        )
        wait_for_success(nav, f"Marker {index}/{total} ({kind})")
        last_distance = last_marker_distance(nav, x, y, sent_ns)
        note = (f", latest Nav2 map pose to recorded marker {last_distance:.2f} m"
                if last_distance is not None else "")
        nav.get_logger().info(f"[REACHED] Marker {index}/{total} ({kind}) SUCCEEDED" + note)
        current = goal
    if home is not None:
        if not nav.nav_to_pose_client.wait_for_server(timeout_sec=5.):
            raise RuntimeError("Return to mapping start: NavigateToPose action unavailable")
        home_goal = make_pose(nav, *home)
        if not nav.goToPose(home_goal):
            raise RuntimeError("Return to mapping start: goal rejected")
        nav.get_logger().info(f"[HOME_GOAL_ACCEPTED] navigating to ({home[0]:.3f}, {home[1]:.3f}) "
                              f"with original map yaw {home[2]:.3f} rad")
        wait_for_success(nav, "Return to mapping start")
        nav.get_logger().info("[HOME_REACHED] Original map start SUCCEEDED")
        nav.get_logger().info(f"[EXAM_COMPLETE] Visited {total} markers in teacher CSV order and returned to start")
        return
    order = "shuffled" if shuffle else "file"
    nav.get_logger().info(f"Visited all {total} saved markers in {order} order")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--marker-file", type=Path, required=True)
    parser.add_argument("--pose-file", type=Path)
    parser.add_argument("--initialize-from-handoff", action="store_true",
                        help="Seed AMCL from the verified four-marker handoff pose")
    parser.add_argument("--approach-distance", type=float, default=REAL_APPROACH_DISTANCE_M,
                        help="Nominal base-to-recorded-marker goal distance in metres (default: 0.40)")
    parser.add_argument("--shuffle", action="store_true", help="Visit a random order distinct from file order for 2–4 landmarks")
    parser.add_argument("--seed", type=int, help="Reproduce a shuffled order")
    parser.add_argument("--list-only", action="store_true", help="Print the visit order and exit before initializing ROS")
    parser.add_argument("--order-file", type=Path, help="Save the selected coordinates and seed to this JSON file")
    parser.add_argument("--order-csv", type=Path, help="Frozen teacher CSV visit sequence")
    parser.add_argument("--return-to-start", action="store_true", help="Finish the teacher route at the original mapping start")
    parser.add_argument("--home-file", type=Path, help="Original marker file's .start_map.txt sidecar")
    parser.add_argument("--start-position-tolerance", type=float, default=.25,
                        help="Maximum departure position difference from the original start in metres (default: 0.25)")
    parser.add_argument("--placement-status", type=Path)
    parser.add_argument("--countdown-request", type=Path)
    parser.add_argument("--placement-generation", type=int)
    parser.add_argument("--rviz-start", action="store_true", help="Wait for an RViz click to start the CSV route")
    options, ros_args = parser.parse_known_args()
    if options.rviz_start and (not options.order_csv or options.placement_status or options.placement_generation or options.countdown_request):
        parser.error("RViz start requires a teacher CSV and manual localization")
    gate = (options.placement_status, options.countdown_request, options.placement_generation)
    if any(v is not None for v in gate) and (not all(gate) or not options.order_csv or options.placement_generation < 1):
        parser.error("Automatic placement requires exam mode, a status file, a countdown request and a positive generation")
    if not math.isfinite(options.approach_distance) or options.approach_distance <= 0:
        parser.error("--approach-distance must be finite and positive")
    if options.initialize_from_handoff and options.shuffle:
        parser.error("The four-marker handoff uses file order")
    if options.seed is not None and not options.shuffle:
        parser.error("--seed requires --shuffle")
    if not (bool(options.order_csv) == options.return_to_start == bool(options.home_file)):
        parser.error("--order-csv, --return-to-start and --home-file must be used together")
    if options.order_csv and (options.shuffle or options.initialize_from_handoff):
        parser.error("Teacher mode uses the supplied CSV order and fresh localization")
    if not math.isfinite(options.start_position_tolerance) or options.start_position_tolerance <= 0:
        parser.error("--start-position-tolerance must be finite and positive")
    if not options.list_only and options.pose_file is None:
        parser.error("--pose-file is required for navigation")
    try:
        path = options.marker_file.expanduser().resolve()
        markers = read_markers(path)
        home = seed = None
        if options.order_csv:
            from exam_marker_order import read_exam_order
            markers = read_exam_order(options.order_csv, markers)
            home = read_home(path, options.home_file.expanduser().resolve())
        else:
            markers, seed = choose_order(markers, options.shuffle, options.seed)
        if options.initialize_from_handoff and len(markers) != 4:
            raise ValueError("四标自动定位交接需要四个已记录的地标。")
        record = {"marker_file": str(path), "shuffle": options.shuffle, "seed": seed,
                  "approach_distance_m": options.approach_distance,
                  "order": [{"index": i, "kind": kind, "x": x, "y": y} for i, (x, y, kind) in enumerate(markers, 1)]}
        if home is not None:
            record.update(order_csv=str(options.order_csv.resolve()), return_to_start=True, home_map_pose=home,
                          home_file=str(options.home_file.resolve()), start_position_tolerance_m=(None if options.rviz_start else options.start_position_tolerance))
        if options.order_file is not None:
            inputs = [p for p in (path, options.order_csv, options.pose_file, options.home_file) if p is not None]
            if options.order_file.expanduser().resolve() in {p.expanduser().resolve() for p in inputs}:
                raise ValueError("保存路线 JSON 的文件不能与地图地标、教师 CSV 或定位输入文件相同。")
            save_order(options.order_file, record)
        print(json.dumps(record, ensure_ascii=False), flush=True)
        if options.list_only:
            return 0
        start = read_map_pose(options.pose_file.expanduser())
    except (OSError, ValueError) as exc:
        print(exc, file=sys.stderr, flush=True)
        return 2

    def interrupt(_signum, _frame):
        raise KeyboardInterrupt

    import rclpy
    from rclpy.signals import SignalHandlerOptions

    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGHUP, interrupt)
    signal.signal(signal.SIGINT, interrupt)
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    nav = navigator()
    try:
        nav.get_logger().info(f"Using localized map pose {start} from {options.pose_file}")
        before_first_goal = None
        if options.rviz_start:
            from rviz_goal_trigger import wait_for_click
            before_first_goal = wait_for_click
        elif options.placement_status:
            def before_first_goal(node):
                wait_for_countdown(*gate, spin=lambda: rclpy.spin_once(node, timeout_sec=.05))
        visit(nav, markers, start, options.shuffle, initialize_from_handoff=options.initialize_from_handoff,
              distance=options.approach_distance, home=home, start_tolerance=options.start_position_tolerance,
              before_first_goal=before_first_goal, manual_start=options.rviz_start)
        return 0
    except PlacementChanged as exc:
        nav.get_logger().warn(str(exc))
        return 75
    except KeyboardInterrupt:
        if nav.result_future is not None and not nav.result_future.done() and nav.goal_handle:
            pending = nav.goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(nav, pending, timeout_sec=2.0)
        return 130
    except (OSError, RuntimeError, ValueError) as exc:
        nav.get_logger().error(str(exc))
        return 1
    finally:
        nav.destroyNode()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
