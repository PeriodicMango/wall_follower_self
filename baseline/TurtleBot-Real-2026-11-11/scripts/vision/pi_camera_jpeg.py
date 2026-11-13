"""Compress the Pi's raw camera stream for the hardware detector when no native JPEG stream exists."""
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def to_bgr(msg):
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid image dimensions {width}x{height}")
    data = np.frombuffer(msg.data, dtype=np.uint8)
    encoding = msg.encoding.lower()
    if encoding in ("nv21", "nv12"):
        if width % 2 or height % 2:
            raise ValueError(f"{encoding} requires even dimensions")
        rows = height + height // 2
        # Some drivers report the total byte count divided by image height as
        # step even though their Y and interleaved chroma planes are tightly packed.
        stride = width if data.size == rows * width else step
        if stride < width or data.size < rows * stride:
            raise ValueError(f"incomplete {encoding} planes ({data.size} bytes, stride {stride})")
        yuv = data[:rows * stride].reshape(rows, stride)[:, :width]
        conversion = cv2.COLOR_YUV2BGR_NV21 if encoding == "nv21" else cv2.COLOR_YUV2BGR_NV12
        return cv2.cvtColor(yuv, conversion)

    formats = {
        "bgr8": (3, None), "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR), "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
        "yuv422_yuy2": (2, cv2.COLOR_YUV2BGR_YUY2), "yuyv": (2, cv2.COLOR_YUV2BGR_YUY2),
        "yuv422": (2, cv2.COLOR_YUV2BGR_UYVY), "uyvy": (2, cv2.COLOR_YUV2BGR_UYVY),
    }
    if encoding not in formats:
        raise ValueError(f"unsupported physical camera encoding {msg.encoding!r}")
    channels, conversion = formats[encoding]
    if step < width * channels or data.size < height * step:
        raise ValueError(f"incomplete {encoding} frame")
    pixels = data[:height * step].reshape(height, step)[:, :width * channels]
    pixels = pixels.reshape(height, width, channels) if channels > 1 else pixels
    return cv2.cvtColor(pixels, conversion) if conversion is not None else pixels


class CameraJpeg(Node):
    def __init__(self):
        super().__init__("hardware_camera_jpeg")
        raw_topic = self.declare_parameter("raw_topic", "/camera/image_raw").value
        jpeg_topic = self.declare_parameter("jpeg_topic", "/turtlebot_test/camera/compressed").value
        self.period = 1 / 8.0
        self.last_frame = -float("inf")
        self.last_error = -float("inf")
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.jpeg = self.create_publisher(CompressedImage, jpeg_topic, qos)
        self.create_subscription(Image, raw_topic, self.callback, qos)
        self.get_logger().info(f"{raw_topic} -> {jpeg_topic}, max 8 Hz, JPEG quality 80")

    def callback(self, msg):
        now = time.monotonic()
        if now - self.last_frame < self.period or self.jpeg.get_subscription_count() == 0:
            return
        self.last_frame = now
        try:
            frame = to_bgr(msg)
            ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            if not ok:
                raise ValueError("OpenCV JPEG encoding failed")
        except (ValueError, cv2.error) as exc:
            if now - self.last_error > 3.0:
                self.get_logger().error(str(exc))
                self.last_error = now
            return
        jpeg = CompressedImage()
        jpeg.header = msg.header
        jpeg.format = "bgr8; jpeg compressed bgr8"
        jpeg.data = encoded.tobytes()
        self.jpeg.publish(jpeg)


def main():
    rclpy.init()
    node = CameraJpeg()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
