"""Run the GitHub map-frame recorder, retaining its hardware pose and output formats."""
import argparse
import os
import sys
import time
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException

SESSION = Path(__file__).resolve().parents[2]
REPO = SESSION / "upstream"
if not REPO.is_dir():  # original development workspace
    REPO = SESSION.parent / "TurtleBot-Sim-2026-11-11/teammate_repo"
sys.path[:0] = [str(REPO / "scripts"), str(REPO)]
import point_transformer as source


class DisplayLandmark(source.Landmark):
    def make_half_marker(self, bottom):
        half = super().make_half_marker(bottom)
        # ROS 2 accepts "map" and the identity quaternion for the unrotated
        # cylinders. All color IDs, hardware display sizes and positions are upstream.
        half.header.frame_id = "map"
        half.pose.orientation.w = 1.0
        return half


class HardwareRecorder(source.PointTransformer):
    def __init__(self):
        super().__init__()
        self.reported = set()
        # The upstream volatile publisher sends a snapshot only when a detection
        # arrives. Refresh it so RViz can join while previously seen markers are
        # out of the camera's field of view. Do not add new measurement samples.
        self.create_timer(1.0, self.refresh_display)

    def point_callback(self, message):
        super().point_callback(message)
        for marker in self.marker_array.markers:
            if marker.ns not in self.reported:
                self.reported.add(marker.ns)
                xy = marker.pose.position
                print(f"[MAPPED] {marker.ns} map=({xy.x:.3f}, {xy.y:.3f}) m", flush=True)

    def refresh_display(self):
        if self.marker_array.markers:
            self.marker_publisher_.publish(self.marker_array)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--marker-file", type=Path, default=SESSION / "maps/markers_real_2026-11-11.txt")
    options, ros_args = parser.parse_known_args()
    marker_file = options.marker_file.expanduser().resolve()
    marker_file.parent.mkdir(parents=True, exist_ok=True)
    os.chdir(marker_file.parent)  # upstream writes landmarks.csv in its current directory
    rclpy.init(args=[*ros_args, "--ros-args", "-p", f"marker_file:={marker_file}"])
    source.Landmark = DisplayLandmark
    node = HardwareRecorder()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.finished_at is not None and (
                (node.start_odom is not None and node.start_map is not None)
                or time.monotonic() - node.finished_at > 5.0
            ):
                break
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.save_landmarks()
        finally:
            node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
