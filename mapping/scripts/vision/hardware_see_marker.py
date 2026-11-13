"""Run the teammate's hardware detector headlessly with the physical camera's transport."""
import argparse
import sys
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

SESSION = Path(__file__).resolve().parents[2]
REPO = SESSION / "upstream"
if not REPO.is_dir():  # original development workspace
    REPO = SESSION.parent / "TurtleBot-Sim-2026-11-11/teammate_repo"
sys.path[:0] = [str(REPO / "scripts"), str(REPO)]
import see_marker as source


class HeadlessHardwareDetector(source.SeeMarker):
    def __init__(self, output):
        self.output = output
        self.output.mkdir(parents=True, exist_ok=True)
        self.last_capture = -float("inf")
        super().__init__()

    def show_images(self, frame, combined_result):
        now = time.monotonic()
        if now - self.last_capture < 1.0:
            return
        self.last_capture = now
        for name, image in (("camera_latest", frame), ("mask_latest", combined_result)):
            temp = self.output / f"{name}.tmp.jpg"
            if cv2.imwrite(str(temp), image):
                temp.replace(self.output / f"{name}.jpg")

    def destroy_node(self):
        # The upstream override closes GUI windows; this process never opens any.
        return Node.destroy_node(self)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--transport", choices=["compressed", "raw"], default="compressed")
    parser.add_argument("--topic", help="Actual camera topic for the selected message type")
    parser.add_argument("--evidence-dir", type=Path, default=SESSION / "evidence/vision")
    options, ros_args = parser.parse_known_args()
    source.USE_COMPRESSED = options.transport == "compressed"
    if options.topic:
        if source.USE_COMPRESSED:
            source.COMPRESSED_TOPIC = options.topic
        else:
            source.RAW_TOPIC = options.topic
    source.SHOW_MASK_WINDOWS = False
    source.FORWARD_OFFSET_CORRECTION = 0.0
    print(f"[硬件视觉] 检测模块 {source.__file__}，前向固定后拉 "
          f"{source.FORWARD_OFFSET_CORRECTION:.2f} m。", flush=True)
    rclpy.init(args=ros_args)
    detector = HeadlessHardwareDetector(options.evidence_dir)
    try:
        rclpy.spin(detector)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        detector.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
