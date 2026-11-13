#!/usr/bin/env python3
"""Recognize the saved mapping start using live LDS data and initialize AMCL."""
import argparse
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.srv import GetParameters
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from tf2_ros import Buffer, TransformException, TransformListener

from capture_rviz_localization import close, pose, stamp
from check_navigation_ready import expected_grid, write_json
from real_run_catalog import csv_rows, finite
from verify_loaded_real_map import compare


MATCH_REASONS = {
    "insufficient valid laser coverage": "有效激光覆盖不足",
    "poor global scan fit": "激光与保存的地图不吻合",
    "ambiguous: too many globally competitive poses": "地图上有多个相似位置",
    "insufficient mapped walls or poor scan fit": "已知墙体覆盖不足或匹配不佳",
    "ambiguous competing map pose": "地图上有两个或更多相似位置",
    "best map pose is outside the saved start": "最符合激光的位置在起点区域外",
}


def covariance_ok(cov):
    # Largest planar covariance eigenvalue includes correlated x/y uncertainty.
    xx, xy, yx, yy, yaw = (cov[i] for i in (0, 1, 6, 7, 35))
    if not all(map(math.isfinite, (xx, xy, yx, yy, yaw))) or min(xx, yy, yaw) < 0:
        return False
    cross = (xy + yx) / 2
    eigen_max = (xx + yy + math.hypot(xx - yy, 2 * cross)) / 2
    return (abs(xy - yx) < 1e-5 and xx * yy + 1e-9 >= cross**2
            and eigen_max <= .30**2 and yaw <= math.radians(25)**2)


def request(path):
    if path is None:
        return {}
    try:
        value = json.loads(path.read_text())
        return value if isinstance(value, dict) else {}
    except (OSError, ValueError):
        return {}


def laser_extrinsic(transform):
    t, q = transform.translation, transform.rotation
    xy_yaw = pose(t, q)
    if xy_yaw is None or math.hypot(q.x, q.y) > .025:
        raise ValueError("激光 TF 不是水平朝上的有效安装位姿")
    return tuple(xy_yaw)


def home_pose(path):
    rows = csv_rows(path)
    if len(rows) != 1 or not finite(rows[0], 3):
        raise ValueError(f"{path} 需要一行建图起点 map x,y,yaw")
    return tuple(map(float, rows[0]))


class StartMonitor(Node):
    def __init__(self, args):
        super().__init__("auto_start_localization", enable_rosout=False, start_parameter_services=False)
        self.args = args
        self.home = home_pose(args.home_file)
        self.meta, self.expected = expected_grid(args.map)
        self.map_matches = False
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.odom = deque(maxlen=180)
        self.scan = self.amcl = None
        self.seed = self.seed_odom = None
        self.generation = 1
        self.measured = None
        self.phase, self.reason = "waiting", "等待真机地图、激光和里程计"
        self.stable_since = self.countdown_since = None
        self.candidate = self.last_good = None
        self.last_processed = -1.
        self.graph_ok = False
        self.clock_ok = False
        self.gids = {}
        self.matched_scan_stamp = 0.
        self.arm_token = None
        self.pending_token = None
        self.blocked_token = request(args.arm_request).get("token")
        self.next_graph = self.next_report = self.next_services = self.next_nomotion = 0.
        self.filename = self.amcl_active = None
        self.nomotion_future = None
        self.nomotion_sent = 0.
        self.svc_tasks = {}
        self.map_param = self.create_client(GetParameters, "/map_server/get_parameters")
        self.amcl_state = self.create_client(GetState, "/amcl/get_state")
        self.nomotion = self.create_client(Empty, "/request_nomotion_update")
        self.seeder = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.create_subscription(OccupancyGrid, "/map", self.on_map,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Odometry, "/odom", self.on_odom, qos_profile_sensor_data)
        self.create_subscription(LaserScan, "/scan", self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.on_amcl, 10)
        self.create_subscription(PoseWithCovarianceStamped, "/initialpose", self.on_initialpose, 10)
        self.pool = ThreadPoolExecutor(max_workers=1)
        self.job = self.matcher = self.laser_pose = None

    def state(self, phase, reason):
        changed = (phase, reason) != (self.phase, self.reason)
        self.phase, self.reason = phase, reason
        if changed:
            print(f"[{time.strftime('%H:%M:%S')}] {reason}", flush=True)
            self.next_report = 0.

    def publish_status(self, force=False):
        now = time.monotonic()
        if force or now >= self.next_report:
            write_json(self.args.watch_status, dict(ready=self.phase == "ready", phase=self.phase,
                       generation=self.generation, reason=self.reason, arm_token=self.arm_token,
                       updated_unix=time.time(), scan_stamp=self.matched_scan_stamp))
            self.next_report = now + .15

    def reset(self, reason):
        if self.stable_since is not None or self.seed is not None or self.measured is not None:
            self.generation += 1
        self.stable_since = self.countdown_since = self.seed = self.seed_odom = None
        self.candidate = self.last_good = self.amcl = self.measured = None
        self.matched_scan_stamp = 0.
        self.arm_token = self.pending_token = None
        self.blocked_token = request(self.args.arm_request).get("token")
        self.args.output.unlink(missing_ok=True)
        self.state("waiting", reason)

    def on_map(self, msg):
        check = compare(msg, self.meta, self.expected)
        self.map_matches = check["all_cells_match"] and check["geometry_matches"]
        if not self.map_matches:
            self.reset("收到的地图与指定地图的原点、尺寸或占用数据不一致")

    def on_scan(self, msg):
        now = time.monotonic()
        t = stamp(msg.header.stamp)
        if self.scan:
            last, old, _ = self.scan
            if t <= old or t - old > .65 or now - last > .65:
                self.reset("激光时间中断，重新确认起点")
        self.scan = now, t, msg

    def on_odom(self, msg):
        now = time.monotonic()
        t = stamp(msg.header.stamp)
        xy_yaw = pose(msg.pose.pose.position, msg.pose.pose.orientation)
        v = msg.twist.twist
        if (not xy_yaw or msg.header.frame_id != "odom" or msg.child_frame_id != "base_footprint"
                or not math.hypot(v.linear.x, v.linear.y) < .025 or not abs(v.angular.z) < .04):
            self.odom.clear()
            self.reset("检测到车身移动或里程计无效，请保持静止")
            return
        if self.odom and (t <= self.odom[-1][1] or t - self.odom[-1][1] > .6 or now - self.odom[-1][0] > .6):
            self.odom.clear()
            self.reset("里程计不连续，重新确认起点")
        anchor = self.seed_odom
        if anchor is not None and not close(xy_yaw, anchor, .025, .05):
            self.odom.clear()
            self.reset("车身已离开本次定位的静止位姿，重新识别")
        self.odom.append((now, t, xy_yaw, msg.header.stamp))

    def on_amcl(self, msg):
        if self.seed is None:
            return
        t = stamp(msg.header.stamp)
        if t < self.seed[1] or msg.header.frame_id != "map" or abs(time.time() - t) > 2:
            return
        xy_yaw = pose(msg.pose.pose.position, msg.pose.pose.orientation)
        cov = [msg.pose.covariance[i] for i in (0, 7, 35)]
        if xy_yaw and covariance_ok(msg.pose.covariance):
            self.amcl = (t, xy_yaw, cov)
        else:
            self.amcl = None
            if self.measured is not None:
                self.reset("AMCL 位姿无效或不确定度过大，重新定位")

    def on_initialpose(self, msg):
        if self.seed is not None and abs(stamp(msg.header.stamp) - self.seed[1]) > 1e-6:
            self.reset("收到另一条初始位姿，重新核实起点")

    def graph(self, now):
        if now < self.next_graph:
            return
        self.next_graph = now + .25
        ids = {}
        expected = {"/map":"map_server", "/scan":"hlds_laser_publisher", "/odom":"diff_drive_controller",
                    "/amcl_pose":"amcl"}
        for topic, name in expected.items():
            pubs = self.get_publishers_info_by_topic(topic)
            if len(pubs) == 1 and pubs[0].node_name == name and pubs[0].node_namespace == "/":
                ids[topic] = bytes(pubs[0].endpoint_gid).hex()
        nodes = self.get_node_names_and_namespaces()
        self.graph_ok = (ids.keys() == expected.keys() and nodes.count(("map_server", "/")) == 1
                         and nodes.count(("amcl", "/")) == 1)
        self.clock_ok = not (self.get_parameter("use_sim_time").value
                             or self.get_publishers_info_by_topic("/clock"))
        changed = [k for k, v in self.gids.items() if ids.get(k) != v]
        if changed:
            self.reset("真机传感器或定位发布者已更换，重新核实")
            self.odom.clear()
            self.scan = None
            if "/map" in changed:
                self.map_matches = False
                self.filename = None
        self.gids = ids

    def poll_checks(self, now):
        for key, (future, _, since) in list(self.svc_tasks.items()):
            if not future.done() and now - since <= 3.:
                continue
            if future.done():
                value = future.result()
                if key == "filename":
                    self.filename = value.values[0].string_value if value and value.values else None
                else:
                    self.amcl_active = bool(value and value.current_state.label == "active")
            else:
                setattr(self, key, None)
                future.cancel()
            del self.svc_tasks[key]
        if now < self.next_services:
            return
        self.next_services = now + 1.
        for key, client, req in (("filename", self.map_param, GetParameters.Request(names=["yaml_filename"])),
                                 ("amcl_active", self.amcl_state, GetState.Request())):
            if key in self.svc_tasks:
                continue
            if client.service_is_ready():
                self.svc_tasks[key] = client.call_async(req), client, now
            else:
                setattr(self, key, None)

    def inputs(self, now):
        if not self.clock_ok or abs(time.time() - self.get_clock().now().nanoseconds * 1e-9) > .5:
            return "检测到仿真时钟或 ROS 时间与系统时间不符，等待真机系统时间"
        if not self.graph_ok:
            return "等待唯一的真机激光、里程计、地图和 AMCL 发布者"
        if not self.map_matches or not self.filename or not Path(self.filename).is_absolute() or Path(self.filename).resolve() != self.args.map:
            return "等待 map_server 确认本轮地图文件及全部栅格"
        if not self.amcl_active:
            return "等待 AMCL 生命周期激活"
        if not self.scan or now - self.scan[0] > .60 or abs(time.time() - self.scan[1]) > .80:
            return "等待新鲜的真机激光"
        if self.scan[2].header.frame_id != "base_scan":
            return "等待 base_scan 坐标系的真机激光"
        if not self.odom or now - self.odom[-1][0] > .55 or abs(time.time() - self.odom[-1][1]) > .65:
            return "等待新鲜的静止里程计"
        recent = [m for m in self.odom if now - m[0] <= 1.5]
        if not recent or recent[-1][0] - recent[0][0] < 1.0:
            return "请保持静止，正在积累连续里程计"
        if not all(close(row[2], recent[0][2], .02, .045) for row in recent):
            return "车身位姿正在变化，请在起点保持静止"
        return None

    def send_seed(self, result):
        if not any(p.node_name == "amcl" and p.node_namespace == "/"
                   for p in self.get_subscriptions_info_by_topic("/initialpose")):
            self.state("matching", "激光已确认起点，等待 AMCL 订阅初始位姿")
            return
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x, msg.pose.pose.position.y = float(result.x), float(result.y)
        msg.pose.pose.orientation.z = math.sin(result.yaw / 2)
        msg.pose.pose.orientation.w = math.cos(result.yaw / 2)
        msg.pose.covariance[0] = msg.pose.covariance[7] = .08 ** 2
        msg.pose.covariance[35] = math.radians(10) ** 2
        self.seed = (time.monotonic(), stamp(msg.header.stamp), [result.x, result.y, result.yaw])
        self.seed_odom = self.odom[-1][2]
        self.seeder.publish(msg)
        self.next_nomotion = 0.
        self.state("seeding", f"激光已稳定识别起点，已提交 AMCL：x={result.x:.2f}、y={result.y:.2f} 米，朝向 {math.degrees(result.yaw):.0f}°")

    def update_amcl(self, now):
        if self.seed is None or now < self.next_nomotion or not self.nomotion.service_is_ready():
            return
        if self.nomotion_future is not None and not self.nomotion_future.done():
            if now - self.nomotion_sent < 2.:
                return
            self.nomotion_future.cancel()
        self.nomotion_future = self.nomotion.call_async(Empty.Request())
        self.nomotion_sent, self.next_nomotion = now, now + .65

    def aligned(self, result):
        if self.amcl is None or abs(time.time() - self.amcl[0]) > 1.8:
            return None
        scan_pose = (result.x, result.y, result.yaw)
        if not close(self.amcl[1], scan_pose, .13, .22):
            return None
        for recv, t, odom, ros_t in reversed(self.odom):
            if time.monotonic() - recv > .55 or abs(time.time() - t) > .65 or t < self.seed[1]:
                continue
            try:
                tf = self.buffer.lookup_transform("map", "base_footprint", Time.from_msg(ros_t))
            except TransformException:
                continue
            xy_yaw = pose(tf.transform.translation, tf.transform.rotation)
            if not xy_yaw or not close(xy_yaw, self.amcl[1], .10, .15) or not close(xy_yaw, scan_pose, .13, .22):
                continue
            if math.hypot(xy_yaw[0]-self.home[0], xy_yaw[1]-self.home[1]) > .25:
                continue
            return dict(generation=self.generation, stamp=t, map_pose=xy_yaw, odom_pose=odom,
                        odom_publisher_gid=self.gids["/odom"], source="automatic_scan_match_amcl",
                        initialpose_stamp=self.seed[1], initial_map_pose=self.seed[2],
                        amcl_stamp=self.amcl[0], amcl_map_pose=self.amcl[1], amcl_xyyaw_variances=self.amcl[2],
                        scan_stamp=self.matched_scan_stamp, scan_match=asdict(result), laser_pose=self.laser_pose)
        return None

    def accept(self, result, sample, now):
        if not result.accepted:
            self.reset("激光尚未唯一匹配建图起点：" + MATCH_REASONS.get(result.reason, "等待可靠的墙体轮廓"))
            return
        xy_yaw = (result.x, result.y, result.yaw)
        if not finite(xy_yaw, 3) or math.hypot(result.x-self.home[0], result.y-self.home[1]) > .25:
            self.reset("匹配位置不在记录起点 0.25 米内")
            return
        if self.candidate is not None and not close(xy_yaw, self.candidate, .08, .16):
            self.reset("激光匹配的位姿或朝向不连续，重新确认")
        if self.last_good is not None and sample - self.last_good > .75:
            self.reset("连续匹配时间中断，重新确认")
        if self.candidate is None:
            self.candidate, self.stable_since = xy_yaw, sample
        self.last_good = self.matched_scan_stamp = sample
        if self.seed is None:
            self.state("matching", "检测到起点，正在持续核对激光与墙体轮廓")
            if sample - self.stable_since >= 2.0:
                self.send_seed(result)
            return
        measured = self.aligned(result)
        if measured is None:
            if self.measured is not None or now - self.seed[0] > 5.:
                self.reset("AMCL、激光与同步地图位姿未能持续吻合，重新定位")
            return
        if self.measured is None:
            self.measured = measured
            write_json(self.args.output, measured)
            self.state("tracking", "AMCL 和激光已确认当前起点。保持静止，等待路线准备")
        countdown = request(self.args.countdown_request)
        if self.args.countdown_request and (type(countdown.get("generation")) is not int
                                            or countdown["generation"] != self.generation):
            self.countdown_since = None
            self.arm_token = self.pending_token = None
            self.state("tracking", "起点定位有效，等待首段路线准备")
            return
        if self.countdown_since is None:
            self.countdown_since = sample
        remaining = 3 - (sample-self.countdown_since)
        if remaining > 0:
            self.state("countdown", f"起点保持稳定，出发倒计时 {math.ceil(remaining)} 秒")
            return
        self.state("ready", "倒计时完成，持续核实激光和定位，等待首个目标放行")
        token_req = request(self.args.arm_request)
        token = token_req.get("token")
        if (not isinstance(token, str) or not token or token == self.blocked_token
                or type(token_req.get("generation")) is not int or token_req["generation"] != self.generation):
            self.arm_token = self.pending_token = None
        elif not self.pending_token or self.pending_token[0] != token:
            self.pending_token = (token, sample)
            self.arm_token = None
        elif sample > self.pending_token[1]:
            self.arm_token = token

    def match_scan(self, scan, extrinsic):
        from start_scan_match import StartScanMatcher
        if self.matcher is None or self.laser_pose != extrinsic:
            self.matcher = StartScanMatcher(self.args.map, self.home, laser_pose=extrinsic, start_tolerance_m=.25)
            self.laser_pose = extrinsic
        return self.matcher.match(scan.ranges, scan.angle_min, scan.angle_increment, scan.range_min, scan.range_max)

    def tick(self):
        now = time.monotonic()
        self.graph(now)
        self.poll_checks(now)
        problem = self.inputs(now)
        if problem:
            self.reset(problem)
            if self.job is not None and self.job[0].done():
                self.job = None
        else:
            self.update_amcl(now)
            if self.job is not None and self.job[0].done():
                future, sample, gen = self.job
                self.job = None
                result = future.result()
                if gen == self.generation and abs(time.time() - sample) <= .80:
                    self.accept(result, sample, now)
            _, t, scan = self.scan
            if self.last_good is not None and time.time() - self.last_good > .8:
                self.reset("等待新鲜的激光匹配结果")
            if self.job is None and t > self.last_processed:
                self.last_processed = t
                try:
                    tf = self.buffer.lookup_transform("base_footprint", scan.header.frame_id, Time.from_msg(scan.header.stamp))
                    extrinsic = laser_extrinsic(tf.transform)
                except (TransformException, ValueError) as exc:
                    self.reset(f"等待真实激光外参 base_footprint → {scan.header.frame_id}（{type(exc).__name__}）")
                else:
                    self.job = self.pool.submit(self.match_scan, scan, extrinsic), t, self.generation
        self.publish_status()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for opt in ("map", "home-file", "output"):
        parser.add_argument("--" + opt, required=True, type=Path)
    parser.add_argument("--timeout", type=float, default=0, help="seconds, 0 waits until cancelled")
    parser.add_argument("--watch-status", type=Path)
    parser.add_argument("--arm-request", type=Path)
    parser.add_argument("--countdown-request", type=Path)
    args = parser.parse_args()
    if args.timeout < 0 or not math.isfinite(args.timeout):
        parser.error("timeout must be finite and nonnegative")
    if (args.arm_request or args.countdown_request) and not args.watch_status:
        parser.error("request files require --watch-status")
    if os.getenv("ROS_DOMAIN_ID") != "3" and os.getenv("ROS_LOCALHOST_ONLY") != "1":
        parser.error("请使用真机 Domain 3 环境，或用独立回环域做离线验证")
    args.map = args.map.expanduser().resolve(strict=True)
    for path in (args.output, args.watch_status):
        if path:
            path.unlink(missing_ok=True)
    node = None
    try:
        def interrupt(_sig, _frame):
            raise KeyboardInterrupt
        for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
            signal.signal(sig, interrupt)
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        node = StartMonitor(args)
        print(f"正在核对地图 {args.map.name} 和真实激光，请将车放至建图起点，朝向不限。", flush=True)
        deadline = time.monotonic() + args.timeout if args.timeout else math.inf
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.025)
            node.tick()
            if not args.watch_status and node.phase == "ready":
                return 0
        print("等待自动起点定位超时。", flush=True)
        return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        return 130
    except (OSError, ValueError, RuntimeError, ImportError) as exc:
        print(f"自动定位停止：{exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if node is not None:
            if args.watch_status or node.phase != "ready":
                node.arm_token = None
                node.state("stopped", "自动起点监测已结束")
                node.publish_status(force=True)
            node.pool.shutdown(wait=False, cancel_futures=True)
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
