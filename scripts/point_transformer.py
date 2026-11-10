#!/usr/bin/env python3
"""Transform detected landmarks into map coordinates and save them after a run."""

import csv
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import MarkerArray

from wall_follower.landmark import marker_type, max_markers, Landmark


def yaw_of(q):
    """Yaw of a geometry_msgs Quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y),
                      1 - 2 * (q.y * q.y + q.z * q.z))


def compose_2d(transform, pose):
    """Apply a TransformStamped to a Pose on the ground plane -> (x, y, yaw).

    Done by hand instead of with tf2_geometry_msgs.do_transform_pose, which
    takes a bare Pose on Humble but a PoseStamped on earlier distros. The
    robot is planar, so roll and pitch are zero and the 2D composition below
    is exact.
    """
    t = transform.transform.translation
    t_yaw = yaw_of(transform.transform.rotation)
    cos_t = math.cos(t_yaw)
    sin_t = math.sin(t_yaw)
    x = t.x + cos_t * pose.position.x - sin_t * pose.position.y
    y = t.y + sin_t * pose.position.x + cos_t * pose.position.y
    return x, y, t_yaw + yaw_of(pose.orientation)


class PointTransformer(Node):
    def __init__(self):
        super().__init__('point_transformer')
        self.marker_file = Path(self.declare_parameter('marker_file', '~/markers.txt').value).expanduser()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.start_odom = None
        self.start_map = None
        self.finished_at = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PoseStamped, 'wall_follower/start_pose', self.start_callback, latched)
        self.create_subscription(Bool, 'wall_follower/finished', self.finish_callback, latched)
        self.create_subscription(PointStamped, '/marker_position', self.point_callback, 10)
        self.marker_publisher_ = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        self.marker_array = MarkerArray()
        self.marker_position = [Landmark(i, self.marker_array.markers) for i in range(max_markers)]
        self.create_timer(0.2, self.capture_start_map)

    def start_callback(self, msg):
        if self.start_odom is None:
            self.start_odom = msg
            self.capture_start_map()

    def finish_callback(self, msg):
        if msg.data and self.finished_at is None:
            self.finished_at = time.monotonic()

    def capture_start_map(self):
        if self.start_odom is None or self.start_map is not None:
            return
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', self.start_odom.header.frame_id, rclpy.time.Time())
        except tf2_ros.TransformException:
            return  # SLAM may not have published map -> odom yet.
        try:
            self.start_map = compose_2d(transform, self.start_odom.pose)
        except Exception as exc:
            # This runs in a timer callback; an escaping exception would kill
            # the recorder and lose the whole run, marker file included.
            self.get_logger().warn(f'Could not place the start pose in map: {exc}')

    def point_callback(self, msg):
        which_marker = int(msg.point.z)
        if which_marker < 0 or which_marker >= max_markers:
            self.get_logger().warn(f'Invalid marker index: {msg.point.z}')
            return
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, stamp)
        except tf2_ros.TransformException:
            try:
                transform = self.tf_buffer.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
            except tf2_ros.TransformException as exc:
                self.get_logger().warn(f'Marker transform unavailable: {exc}')
                return
        # point.z encodes the type index, not a camera-space height.
        point = PointStamped()
        point.header = msg.header
        point.point.x = msg.point.x
        point.point.y = msg.point.y
        map_point = tf2_geometry_msgs.do_transform_point(point, transform)
        landmark = self.marker_position[which_marker]
        landmark.update_position(map_point.point)
        # newest reading is always the closest one; overwrite the running mean
        # landmark.py keeps, which weights by frame count rather than accuracy
        landmark.top_marker.pose.position.x = map_point.point.x
        landmark.top_marker.pose.position.y = map_point.point.y
        landmark.bottom_marker.pose.position.x = map_point.point.x
        landmark.bottom_marker.pose.position.y = map_point.point.y
        self.marker_publisher_.publish(self.marker_array)

    def save_landmarks(self):
        # Preserve the original assignment output for older scripts.
        with open('landmarks.csv', 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            for lm in self.marker_position:
                if lm.top_marker is not None:
                    writer.writerow(lm.toRow())

        if self.start_odom is None:
            raise RuntimeError('No wall_follower/start_pose received; cannot write marker file')

        # The assignment's marker file is written first and unconditionally:
        # nothing optional below may be able to stop it from reaching disk.
        pose = self.start_odom.pose
        found = [lm for lm in self.marker_position if lm.top_marker is not None]
        with self.marker_file.open('w', newline='') as stream:
            writer = csv.writer(stream)
            writer.writerow([pose.position.x, pose.position.y, yaw_of(pose.orientation)])
            for lm in found:
                p = lm.top_marker.pose.position
                writer.writerow([p.x, p.y, marker_type[lm.mtype]])
        self.get_logger().info(f'Saved {len(found)} markers to {self.marker_file}')
        if len(found) != 4:
            self.get_logger().warn(f'Expected 4 markers, found {len(found)}')

        # The map-frame start pose is an extra, not part of the required format.
        # navigate_markers falls back to the odom start row when it is absent,
        # so a sidecar left over from an earlier run must not survive this one.
        sidecar = self.marker_file.with_suffix('.start_map.txt')
        if self.start_map is None:
            sidecar.unlink(missing_ok=True)
            self.get_logger().warn(
                'No map -> odom transform; wrote no map-frame start pose. '
                'Navigation will read the odom start row as a map pose.')
            return
        with sidecar.open('w', newline='') as stream:
            csv.writer(stream).writerow(self.start_map)
        self.get_logger().info(f'Saved the map-frame start pose to {sidecar}')


def main(args=None):
    rclpy.init(args=args)
    node = PointTransformer()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.finished_at is not None and (
                (node.start_odom is not None and node.start_map is not None)
                or time.monotonic() - node.finished_at > 5.0
            ):
                break
    except KeyboardInterrupt:
        pass
    try:
        node.save_landmarks()
    except (OSError, RuntimeError) as exc:
        node.get_logger().error(str(exc))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
