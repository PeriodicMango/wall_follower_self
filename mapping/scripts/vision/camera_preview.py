"""Display compressed images already published by the .103 camera on the Mac."""

from datetime import datetime
from time import monotonic

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage


TOPIC = "/turtlebot_test/camera/compressed"
WINDOW = "TurtleBot .103 camera"


def main():
    rclpy.init()
    node = rclpy.create_node("robot103_camera_preview", enable_rosout=False)
    last_frame = None
    stale = False

    def show(message):
        nonlocal last_frame, stale
        if not message.data:
            return
        frame = cv2.imdecode(np.frombuffer(message.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        cv2.imshow(WINDOW, frame)
        if last_frame is None or stale:
            height, width = frame.shape[:2]
            cv2.setWindowTitle(WINDOW, f"{WINDOW} | LIVE | {width} x {height}")
            print(f"{datetime.now():%H:%M:%S}  已解码并显示 {width} × {height} 实机图像，载荷 {len(message.data)} 字节。")
        last_frame = monotonic()
        stale = False

    node.create_subscription(
        CompressedImage, TOPIC, show, QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    )
    try:
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW, 800, 600)
        cv2.setWindowTitle(WINDOW, f"{WINDOW} | Waiting for JPEG")
        print(f"等待 {TOPIC}，按 Q、Esc 或关闭窗口即可退出。")
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            key = cv2.waitKey(10) & 0xFF
            if key in (27, ord("q"), ord("Q")) or cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                break
            if last_frame is not None and monotonic() - last_frame > 2 and not stale:
                stale = True
                cv2.setWindowTitle(WINDOW, f"{WINDOW} | Waiting for new frames")
                print(f"{datetime.now():%H:%M:%S}  两秒内未收到可解码的新图像。")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
