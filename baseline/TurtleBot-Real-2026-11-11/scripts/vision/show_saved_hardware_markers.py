"""Display a completed hardware run's saved map coordinates after its recorder exits."""

import argparse
import csv
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import MarkerArray

from hardware_marker_recorder import DisplayLandmark, source


TOPIC = "/visualization_marker_array"


def load_markers(path):
    with path.open(newline="") as stream:
        rows = [row for row in csv.reader(stream) if row]
    if not rows or len(rows[0]) != 3 or not all(math.isfinite(float(s)) for s in rows[0]):
        raise ValueError("首行需要包含建图起点的 x、y 和 yaw。")
    array = MarkerArray()
    seen = set()
    for number, row in enumerate(rows[1:], 2):
        if len(row) != 3:
            raise ValueError(f"第 {number} 行需要包含 x、y 和色序。")
        x, y = map(float, row[:2])
        kind = row[2].strip()
        if kind not in source.marker_type or kind in seen or not all(map(math.isfinite, (x, y))):
            raise ValueError(f"第 {number} 行包含无效坐标或色序。")
        seen.add(kind)
        landmark = DisplayLandmark(source.marker_type.index(kind), array.markers)
        landmark.update_position(Point(x=x, y=y))
    return array


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--marker-file", type=Path, required=True)
    options, ros_args = parser.parse_known_args()
    path = options.marker_file.expanduser()
    try:
        markers = load_markers(path)
    except (OSError, ValueError) as exc:
        parser.exit(1, f"{path} — {exc}\n")
    if not markers.markers:
        parser.exit(0, "文件里只有起点，尚无实测地标可显示。\n")
    rclpy.init(args=ros_args)
    node = rclpy.create_node("saved_hardware_landmarks")
    publisher = None

    def display():
        nonlocal publisher
        other = any(p.node_name != node.get_name() or p.node_namespace != node.get_namespace()
                    for p in node.get_publishers_info_by_topic(TOPIC))
        if other:
            if publisher is not None:
                node.destroy_publisher(publisher)
                publisher = None
                node.get_logger().info("实时地标发布者已上线，停止刷新文件地标。")
            return
        if publisher is None:
            publisher = node.create_publisher(MarkerArray, TOPIC, 1)
            node.get_logger().info(f"正在显示 {path} 中的 {len(markers.markers)//2} 个地标。")
        publisher.publish(markers)

    node.create_timer(1.0, display)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
