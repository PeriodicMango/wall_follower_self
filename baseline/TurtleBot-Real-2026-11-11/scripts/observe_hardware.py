#!/usr/bin/env python3
"""Observe TurtleBot topics. Optionally capture a stationary SLAM-to-AMCL handoff."""
import argparse
import csv
import json
import math
import re
import time
from collections import Counter, deque
from pathlib import Path
from tempfile import TemporaryDirectory

import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CompressedImage, Image, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import MarkerArray

KINDS = ('yellow/pink', 'green/pink', 'blue/pink', 'pink/yellow', 'pink/green', 'pink/blue')
ROOT = Path(__file__).resolve().parents[1]
GOAL = re.compile(r'Marker (\d+)/4 \(([^)]+)\) at .*; navigating to .*')


def sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def xy_yaw(p, q):
    return [p.x, p.y, math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))]


def endpoints(infos):
    return [dict(node=i.node_namespace.rstrip('/') + '/' + i.node_name, gid=bytes(i.endpoint_gid).hex())
            for i in infos]


def load_markers(path):
    with path.open(newline='') as stream:
        rows = [row for row in csv.reader(stream) if row]
    if len(rows) != 5 or any(len(row) != 3 for row in rows):
        raise ValueError('Expected start and four marker rows')
    points = {row[2].strip(): [float(row[0]), float(row[1])] for row in rows[1:]}
    if len(points) != 4 or not points.keys() <= set(KINDS):
        raise ValueError('Expected four distinct SeeMarker color labels')
    if not all(math.isfinite(v) for point in points.values() for v in point):
        raise ValueError('Non-finite marker coordinate')
    return rows, points


class Monitor(Node):
    def __init__(self, a):
        super().__init__('turtlebot_handoff_observer' if a.handoff else 'turtlebot_hardware_observer')
        self.a = a
        self.started = self.get_clock().now().nanoseconds * 1e-9
        self.count = Counter()
        self.point_counts = Counter()
        self.latest, self.received, self.mapped, self.actions, self.announced = {}, {}, {}, {}, {}
        self.guard_log = None
        self.completed = False
        self.camera = None
        self.image_sub = None
        self.odom_window = deque(maxlen=500)
        if a.resume_state:
            previous = json.loads(a.resume_state.read_text())
            elapsed = max(0.0, time.time() - previous['unix'])
            self.count.update(previous['counts'])
            self.point_counts.update(previous['raw_detector_counts'])
            self.latest.update(previous['latest'])
            self.received.update({t: time.monotonic()-elapsed-age for t,age in previous['ages_s'].items()})
            self.mapped.update(previous['mapped_marker_xy'])
            self.actions.update(previous['goals'])
            self.announced.update({int(i): kind for i, kind in previous['announced'].items()})
            self.completed = previous['mission_reported_all_four']
            self.guard_log = previous['guard'].get('last_rosout')
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self.odom, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, '/map', self.grid,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(PointStamped, '/marker_position', self.point, 10)
        self.create_subscription(MarkerArray, '/visualization_marker_array', self.markers, 10)
        self.create_subscription(Log, '/rosout', self.log, 50)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.status, 10)
        for topic in dict.fromkeys(['/cmd_vel', *a.command_input]):
            self.create_subscription(Twist, topic, lambda m, t=topic: self.seen(t, [m.linear.x, m.angular.z]),
                                     qos_profile_sensor_data)

    def seen(self, topic, value):
        self.count[topic] += 1
        self.received[topic] = time.monotonic()
        self.latest[topic] = value

    def scan(self, m):
        d = [x for x in m.ranges if math.isfinite(x) and m.range_min <= x <= m.range_max]
        self.seen('/scan', dict(stamp=sec(m.header.stamp), frame=m.header.frame_id, count=len(m.ranges),
                                valid=len(d), nearest_m=min(d) if d else None, limits=[m.range_min, m.range_max]))

    def odom(self, m):
        p = xy_yaw(m.pose.pose.position, m.pose.pose.orientation)
        v = m.twist.twist
        item = dict(stamp=sec(m.header.stamp), frame=m.header.frame_id, child=m.child_frame_id,
                    pose=p, speed=[math.hypot(v.linear.x, v.linear.y), v.angular.z])
        self.odom_window.append((time.monotonic(), m.header.stamp, item))
        self.seen('/odom', item)

    def grid(self, m):
        self.seen('/map', dict(frame=m.header.frame_id, stamp=sec(m.header.stamp), width=m.info.width,
                               height=m.info.height, resolution=m.info.resolution,
                               known=sum(cell >= 0 for cell in m.data)))

    def discover_camera(self):
        if self.image_sub is not None:
            return
        candidates = []
        for topic, types in self.get_topic_names_and_types():
            if self.a.camera and topic != self.a.camera:
                continue
            for cls in (CompressedImage, Image):
                if 'sensor_msgs/msg/' + cls.__name__ in types and self.get_publishers_info_by_topic(topic):
                    candidates.append((cls != CompressedImage, 'depth' in topic, topic, cls))
        if candidates:
            _, _, self.camera, cls = min(candidates)
            self.image_sub = self.create_subscription(cls, self.camera, self.image, qos_profile_sensor_data)

    def image(self, m):
        self.seen(self.camera, dict(stamp=sec(m.header.stamp), frame=m.header.frame_id, bytes=len(m.data),
                                   format=m.format if isinstance(m, CompressedImage) else m.encoding))

    def point(self, m):
        z = m.point.z
        if all(map(math.isfinite, [m.point.x, m.point.y, z])) and z.is_integer() and 0 <= z < len(KINDS):
            self.point_counts[KINDS[int(z)]] += 1
        self.seen('/marker_position', dict(stamp=sec(m.header.stamp), frame=m.header.frame_id))

    def markers(self, m):
        for mark in m.markers:
            p = mark.pose.position
            if mark.ns in KINDS and mark.header.frame_id.lstrip('/') == 'map' and mark.action == mark.ADD:
                if math.isfinite(p.x) and math.isfinite(p.y):
                    self.mapped[mark.ns] = [p.x, p.y]
        self.seen('/visualization_marker_array', dict(self.mapped))

    def log(self, m):
        if m.name.split('.')[-1] == self.a.guard_node.strip('/'):
            self.guard_log = {'text': m.msg, 'stamp': sec(m.stamp)}
        if m.name.split('.')[-1] == 'navigate_markers':
            match = GOAL.fullmatch(m.msg)
            if match:
                self.announced[int(match[1])] = match[2]
            if m.msg == 'Visited all four markers in file order':
                self.completed = True

    def status(self, m):
        for item in m.status_list:
            stamp = sec(item.goal_info.stamp)
            if stamp >= self.started:
                uid = bytes(item.goal_info.goal_id.uuid).hex()
                prior = self.actions.get(uid, {})
                if prior.get('status') == item.status:
                    continue
                data = dict(accepted_stamp=stamp, status=item.status)
                if item.status in (4, 5, 6):
                    data['terminal_map_pose'] = self.map_pose()
                self.actions[uid] = data

    def map_pose(self):
        try:
            tf = self.tf.lookup_transform('map', 'base_footprint', Time())
            return dict(stamp=sec(tf.header.stamp), pose=xy_yaw(tf.transform.translation, tf.transform.rotation))
        except TransformException:
            return None

    def guard(self):
        data = {'last_rosout': self.guard_log}
        if self.a.guard_status:
            try:
                s = json.loads(self.a.guard_status.read_text())
                data.update(status=s, age_s=time.time()-s['updated_at_unix'])
            except (ValueError, OSError, KeyError) as e:
                data['status_file_error'] = str(e)
        return data

    def report(self):
        self.discover_camera()
        topics = ['/scan', '/odom', '/cmd_vel', *self.a.command_input]
        if self.camera:
            topics.append(self.camera)
        graph = {t: dict(publishers=endpoints(self.get_publishers_info_by_topic(t)),
                         subscribers=endpoints(self.get_subscriptions_info_by_topic(t))) for t in topics}
        points = {}
        marker_error = None
        if self.a.markers:
            try:
                _, points = load_markers(self.a.markers)
            except (OSError, ValueError) as exc:
                marker_error = str(exc)
        ordered = [v for _,v in sorted(self.actions.items(), key=lambda i: i[1]['accepted_stamp'])]
        events = [self.announced.get(i) for i in range(1, 5)]
        coordinate_differences = {kind: math.dist(self.mapped[kind], xy)
                                  for kind, xy in points.items() if kind in self.mapped}
        state = dict(unix=time.time(), camera_topic=self.camera, counts=dict(self.count), latest=self.latest,
                     ages_s={t: time.monotonic()-v for t,v in self.received.items()}, graph=graph,
                     raw_detector_counts=dict(self.point_counts), mapped_marker_xy=self.mapped,
                     four_measured_and_saved=(len(coordinate_differences) == 4
                                              and points.keys() <= self.point_counts.keys()
                                              and max(coordinate_differences.values()) < .02),
                     file_vs_observed_map_distance_m=coordinate_differences,
                     recorded_marker_xy=points, marker_file_error=marker_error, guard=self.guard(),
                     current_map_pose=self.map_pose(), goals=self.actions, announced=self.announced,
                     mission_reported_all_four=self.completed,
                     four_goals_succeeded=(self.completed and len(ordered)==4 and len(points)==4
                                           and events==list(points) and all(a['status']==4 for a in ordered)))
        temp = self.a.output.with_suffix('.tmp')
        temp.write_text(json.dumps(state, indent=2)+'\n')
        temp.replace(self.a.output)

    def handoff(self):
        rows, _ = load_markers(self.a.markers)
        now = time.monotonic()
        stationary = [r for r in self.odom_window if now-r[0] <= 2.5]
        if not stationary or stationary[-1][0]-stationary[0][0] < 2.0 or now-stationary[-1][0] > .4:
            raise ValueError('Insufficient recent odometry for the stationary handoff')
        x0, y0, a0 = stationary[0][2]['pose']
        for _, _, obs in stationary:
            x, y, a = obs['pose']
            if (obs['frame'] != 'odom' or obs['child'] != 'base_footprint'
                or max(abs(v) for v in obs['speed']) > .025 or math.hypot(x-x0, y-y0) > .015
                or abs(math.remainder(a-a0, 2*math.pi)) > .035):
                raise ValueError('Robot moved or odometry frames differ from odom/base_footprint')
        for _, stamp, obs in reversed(stationary):
            if abs(time.time()-sec(stamp)) > .6:
                continue
            try:
                tf = self.tf.lookup_transform('map', 'base_footprint', Time.from_msg(stamp))
                break
            except TransformException:
                continue
        else:
            raise ValueError('No recent matching map/base transform at a measured odometry stamp')
        pose = xy_yaw(tf.transform.translation, tf.transform.rotation)
        odom_sources = endpoints(self.get_publishers_info_by_topic('/odom'))
        if len(odom_sources) != 1 or odom_sources[0]['node'] != '/diff_drive_controller':
            raise ValueError('Could not identify the physical odometry publisher for handoff')
        dest = self.a.handoff
        # Publish the navigation copy and its sidecar as one new directory.
        if dest.exists():
            raise FileExistsError(dest)
        dest.parent.mkdir(parents=True, exist_ok=True)
        with TemporaryDirectory(dir=dest.parent) as tmp:
            folder = Path(tmp) / 'handoff'
            folder.mkdir()
            path = folder / self.a.markers.name
            with path.open('w', newline='') as stream:
                csv.writer(stream).writerows([obs['pose'], *rows[1:]])
            with path.with_suffix('.start_map.txt').open('w', newline='') as stream:
                csv.writer(stream).writerow(pose)
            (folder / 'measured_pose.json').write_text(json.dumps(
                dict(stamp=sec(stamp), map_pose=pose, odom_pose=obs['pose'],
                     odom_publisher_gid=odom_sources[0]['gid'],
                     source_marker_file=str(self.a.markers.resolve())), indent=2)+'\n')
            folder.rename(dest)
        print('Navigation handoff:', dest / self.a.markers.name, flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path)
    p.add_argument('--markers', type=Path)
    p.add_argument('--guard-status', type=Path, help='Local or mirrored robot guard JSON')
    p.add_argument('--resume-state', type=Path, help='Preserve counts and evidence from this run on observer restart')
    p.add_argument('--guard-node', default='turtlebot_test_velocity_guard')
    p.add_argument('--command-input', action='append', default=None)
    p.add_argument('--camera', help='Image or CompressedImage topic; autodetects when omitted')
    p.add_argument('--duration', type=float, help='Seconds to observe; default continuous')
    p.add_argument('--handoff', type=Path, help='After observing, save measured poses into a NEW directory')
    a = p.parse_args()
    if a.output is None:
        stem = 'handoff_status' if a.handoff else 'hardware_status'
        a.output = ROOT / 'evidence' / (stem + '_2026-11-11.json')
    a.command_input = a.command_input or ['/turtlebot_test/wall_cmd_vel']
    if a.handoff and not (a.markers and a.duration and a.duration >= 4):
        p.error('--handoff requires --markers and --duration of at least 4 seconds')
    a.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = Monitor(a)
    started = next_report = time.monotonic()
    try:
        while rclpy.ok() and (a.duration is None or time.monotonic()-started < a.duration):
            rclpy.spin_once(node, timeout_sec=.1)
            if time.monotonic() >= next_report:
                node.report()
                next_report = time.monotonic()+1
        if rclpy.ok():
            node.report()
            if a.handoff:
                node.handoff()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
