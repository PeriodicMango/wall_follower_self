#!/usr/bin/env python3
"""Bounded TurtleBot test gate. No motion until ARM contains a fresh token."""

import argparse
import json
import math
import os
from pathlib import Path
import re
import signal
import time


class Guard:
    MAX_LINEAR = 0.06
    NAV_FORWARD = 0.13
    MAX_ANGULAR = 0.30
    MAX_SECONDS = 30.0

    def __init__(self, max_seconds=30.0):
        self.max_seconds = max_seconds
        self.received = {"scan": None, "odom": None, "command": None}
        self.errors = {"scan": "not received", "odom": "not received", "command": "not received"}
        self.counts = {key: 0 for key in self.received}
        self.rejected = {key: 0 for key in self.received}
        self.last_stamp = {"scan": None, "odom": None}
        self.command = (0.0, 0.0)
        self.minimum_range = None
        self.seen_tokens = set()
        self.armed_at = None
        self.mode = "disarmed"
        self.arm_state = "awaiting new ARM token"

    def _reject(self, kind, reason):
        self.errors[kind] = reason
        self.rejected[kind] += 1

    def _stamp_advances(self, kind, stamp):
        previous = self.last_stamp[kind]
        if stamp <= 0 or (previous is not None and stamp <= previous):
            self._reject(kind, "timestamp not advancing")
            return False
        self.last_stamp[kind] = stamp
        return True

    def scan(self, ranges, angle_min, angle_increment, range_min, range_max, stamp, now):
        self.counts["scan"] += 1
        metadata = (angle_min, angle_increment, range_min, range_max)
        if (not all(math.isfinite(x) for x in metadata)
                or len(ranges) != 360 or abs(angle_min) > 0.005
                or abs(angle_increment - math.tau / 360) > 0.0001
                or not 0 <= range_min < range_max):
            self._reject("scan", "unsupported scan geometry or range limits")
            return
        if not self._stamp_advances("scan", stamp):
            return
        valid = [x for x in ranges if math.isfinite(x) and range_min <= x <= range_max]
        if not valid:
            self.minimum_range = None
            self._reject("scan", "all laser ranges invalid")
            return
        self.minimum_range = min(valid)
        self.received["scan"] = now
        self.errors["scan"] = None

    def odom(self, stamp, values, now):
        self.counts["odom"] += 1
        if not all(math.isfinite(x) for x in values):
            self._reject("odom", "non-finite odometry")
            return
        if self._stamp_advances("odom", stamp):
            self.received["odom"] = now
            self.errors["odom"] = None

    def velocity(self, values, now):
        self.counts["command"] += 1
        if len(values) != 6 or not all(math.isfinite(x) for x in values):
            self._reject("command", "non-finite or malformed command")
            return
        self.command = (values[0], values[5])
        self.received["command"] = now
        self.errors["command"] = None

    def output(self, now, token):
        # A token observed while running cannot extend or restart that run.
        new_token = bool(token) and token not in self.seen_tokens
        if new_token:
            self.seen_tokens.add(token)
        if self.armed_at is not None:
            if not token:
                self.armed_at = None
                self.arm_state = "ARM removed; new token required"
            elif self.max_seconds is not None and now - self.armed_at >= self.max_seconds:
                self.armed_at = None
                self.arm_state = "30 second test complete; new token required"
        elif new_token:
            self.armed_at = now
            self.arm_state = "armed"
            self.mode = "navigation" if re.fullmatch(r"navigation:[0-9a-f]{32}", token) else "mapping"

        if self.armed_at is None:
            self.mode = "disarmed"
        forward_limit = self.NAV_FORWARD if self.mode == "navigation" else self.MAX_LINEAR

        reasons = []
        if self.armed_at is None:
            reasons.append(self.arm_state)
        for kind, timeout in (("scan", 0.6), ("odom", 0.6), ("command", 0.3)):
            if self.errors[kind]:
                reasons.append(f"{kind}: {self.errors[kind]}")
            elif self.received[kind] is None or now - self.received[kind] >= timeout:
                reasons.append(f"{kind}: stale")
        if reasons:
            command = (0.0, 0.0)
        else:
            command = (max(-self.MAX_LINEAR, min(forward_limit, self.command[0])),
                       max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, self.command[1])))
        status = {
            "guard_schema_version": 2,
            "mode": self.mode,
            "forward_limit_m_s": forward_limit,
            "navigation_forward_limit_m_s": self.NAV_FORWARD,
            "state": "holding_zero" if reasons else "armed_ready",
            "reasons": reasons,
            "counts": dict(self.counts),
            "rejected": dict(self.rejected),
            "age_seconds": {key: None if val is None else round(now - val, 3)
                            for key, val in self.received.items()},
            "minimum_valid_range_m": self.minimum_range,
            "remaining_seconds": 0.0 if self.armed_at is None else (None if self.max_seconds is None else round(max(0, self.max_seconds - (now - self.armed_at)), 3)),
            "output": {"linear_x": command[0], "angular_z": command[1]},
        }
        return command, status


def read_token(path):
    try:
        with path.open("r", encoding="utf-8") as stream:
            token = stream.read(4097)
        return token.strip() if len(token) <= 4096 else None
    except (OSError, UnicodeError):
        return None


def write_status(path, status):
    status["updated_at_unix"] = time.time()
    temporary = path.with_name(path.name + f".{os.getpid()}.tmp")
    temporary.write_text(json.dumps(status, indent=2, allow_nan=False) + "\n")
    os.replace(temporary, path)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    folder = Path(__file__).resolve().parent
    parser.add_argument("--arm-file", type=Path, default=folder / "ARM")
    parser.add_argument("--status-file", type=Path, default=folder / "guard_status.json")
    parser.add_argument("--continuous", action="store_true", help="Run until disarmed; keep sensor and command watchdogs")
    args = parser.parse_args()

    import rclpy
    from rclpy.clock import Clock, ClockType
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import LaserScan

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("turtlebot_test_velocity_guard")
    node.set_parameters([Parameter("use_sim_time", value=False)])
    publisher = node.create_publisher(Twist, "/cmd_vel", 1)
    guard = Guard(max_seconds=None if args.continuous else 30.0)
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def stamp(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def scan_callback(message):
        guard.scan(message.ranges, message.angle_min, message.angle_increment,
                   message.range_min, message.range_max, stamp(message), time.monotonic())

    def odom_callback(message):
        p, q, v = message.pose.pose.position, message.pose.pose.orientation, message.twist.twist
        guard.odom(stamp(message), (p.x, p.y, p.z, q.x, q.y, q.z, q.w,
                                   v.linear.x, v.linear.y, v.linear.z,
                                   v.angular.x, v.angular.y, v.angular.z), time.monotonic())

    def command_callback(message):
        guard.velocity((message.linear.x, message.linear.y, message.linear.z,
                        message.angular.x, message.angular.y, message.angular.z), time.monotonic())

    node.create_subscription(LaserScan, "/scan", scan_callback, qos_profile_sensor_data)
    node.create_subscription(Odometry, "/odom", odom_callback, qos_profile_sensor_data)
    node.create_subscription(Twist, "/turtlebot_test/wall_cmd_vel", command_callback, 1)
    last_status_at = float("-inf")
    last_reason = None
    status = {"state": "starting", "reasons": []}

    def tick():
        nonlocal last_status_at, last_reason, status
        now = time.monotonic()
        command, status = guard.output(now, read_token(args.arm_file))
        message = Twist()
        message.linear.x, message.angular.z = command
        publisher.publish(message)
        reason = "; ".join(status["reasons"]) or "armed, sensors fresh"
        if reason != last_reason:
            node.get_logger().info(reason)
            last_reason = reason
        if now - last_status_at >= 1.0:
            write_status(args.status_file, status)
            last_status_at = now

    node.create_timer(0.05, tick, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        while not stopping and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        # Keep the context alive to deliver repeated zero commands before exit.
        if rclpy.ok():
            for _ in range(10):
                publisher.publish(Twist())
                time.sleep(0.03)
        status.update(state="stopped", mode="disarmed", forward_limit_m_s=guard.MAX_LINEAR, reasons=["guard exited"],
                      output={"linear_x": 0.0, "angular_z": 0.0}, remaining_seconds=0.0)
        try:
            write_status(args.status_file, status)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
