#!/usr/bin/env python3

# Released under GPLv3: https://www.gnu.org/licenses/gpl-3.0.html
# Author: Claude Sammut
# Merged version: see_marker(3) + compatible improvements from teammate see_marker 2
#
# 【保留自 (3)】
# - HSV CLICK：点击 camera 窗口输出真实 H/S/V
# - 当前现场正在使用的 HSV 阈值
# - 四色 mask 调试窗口 + BLOB / MARKER 日志
# - 较严格的小噪点过滤 (area >= 100, h >= 10)
# - 上下两块高度比例检查
# - 使用老师 landmark.py 的 marker_type 顺序，避免下游类型编号错位
#
# 【合并自队友 2】
# - 支持 /camera/image_raw/compressed
# - Camera QoS: BEST_EFFORT + KEEP_LAST depth=1，避免积压旧帧
# - 标定距离函数: 39.2644 * h^(-0.9413)
# - 用实际 image width 计算图像中心/角度
# - bearing 使用正负角：左正、右负，不再把右侧转换成 360-angle
# - 增加上下色块纵向配对检查
# - 使用 Marker 上半块对应的 distance/angle
# - 每帧只发布一个有效 Marker
# - combined result 窗口及更稳妥的节点退出清理

import math
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry

from wall_follower.landmark import marker_type


# ==========================================================
# Camera input
# True  = teammate's compressed stream (recommended on robot Wi-Fi)
# False = RAW /camera/image_raw fallback
# ==========================================================
USE_COMPRESSED = True

RAW_TOPIC = '/camera/image_raw'
COMPRESSED_TOPIC = '/camera/image_raw/compressed'

FIELD_OF_VIEW_H = 62.2

# 【保留 (3)】当前现场阈值；不要用队友文件的另一套值覆盖你正在调好的版本。
COLOURS = {
    # 实测：H=164, S=159, V=210
    'pink': (
        (155, 90, 100),
        (175, 255, 255)
    ),

    # 实测：H=101, S=245, V=183
    'blue': (
        (95, 140, 80),
        (110, 255, 255)
    ),

    # 当前 (3) 中的值；有真实 Green 数据后可继续收紧
    'green': (
        (40, 100, 70),
        (85, 255, 255)
    ),

    # 当前 (3) 中的值；有真实 Yellow 数据后可继续收紧
    'yellow': (
        (20, 120, 100),
        (38, 255, 255)
    )
}

# 【保留 (3)】比队友版 MIN_AREA=13 / MIN_HEIGHT=3 更严格，
# 用于过滤你之前看到的 h=2~5 / area=2~11 黄色噪点。
MIN_HEIGHT = 10
MIN_AREA = 100

# 上下两半的高度应大致接近。
MIN_HEIGHT_RATIO = 0.5
MAX_HEIGHT_RATIO = 1.5

# 是否显示四种单独 mask；正式跑如觉得窗口太多可改 False。
SHOW_MASK_WINDOWS = True

# ==========================================================
# 【中文修改标注：自适应转弯过滤 + 画面边缘过滤】
#
# 注意：这里不要求 Marker 正对。
#
# 正常情况：
#   |angular_z| <= 0.25 -> 直接允许
#
# 中等转弯：
#   0.25 < |angular_z| <= 0.45
#   如果 Marker 不靠画面边缘、上下色块比例正常 -> 仍允许
#
# 快速转弯：
#   |angular_z| > 0.45 -> 一律跳过
#
# 这样既避免严格 gate 完全漏掉 Marker，
# 又不依赖“必须正对”的条件。
# ==========================================================
NORMAL_MAX_ANGULAR_SPEED = 0.25
EMERGENCY_MAX_ANGULAR_SPEED = 0.45
EDGE_MARGIN_RATIO = 0.08
TURN_MIN_HEIGHT_RATIO = 0.75
TURN_MAX_HEIGHT_RATIO = 1.25


# ==========================================================
# 【保留 (3)】点击 camera 窗口读取 HSV
# ==========================================================
_last_hsv_frame = None


def show_hsv(event, x, y, flags, param):
    global _last_hsv_frame

    if event == cv2.EVENT_LBUTTONDOWN and _last_hsv_frame is not None:
        if 0 <= y < _last_hsv_frame.shape[0] and 0 <= x < _last_hsv_frame.shape[1]:
            h, s, v = _last_hsv_frame[y, x]
            print(
                f'[HSV CLICK] x={x}, y={y} -> '
                f'H={int(h)}, S={int(s)}, V={int(v)}',
                flush=True
            )


# ==========================================================
# RAW fallback decoder：只有 USE_COMPRESSED=False 时使用。
# ==========================================================
def decode_raw_image(data, bridge):
    encoding = (data.encoding or '').lower()

    if encoding == 'nv21':
        width = int(data.width)
        height = int(data.height)
        raw = np.frombuffer(data.data, dtype=np.uint8)
        expected = width * height * 3 // 2

        if raw.size < expected:
            raise ValueError(
                f'NV21 buffer too small: got {raw.size} bytes, '
                f'expected at least {expected}'
            )

        raw = raw[:expected]
        nv21 = raw.reshape((height * 3 // 2, width))
        return cv2.cvtColor(nv21, cv2.COLOR_YUV2BGR_NV21)

    return bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')


class SeeMarker(Node):

    def __init__(self):
        super().__init__('see_marker')

        self.br = CvBridge()
        self._printed_camera_meta = False

        # 【中文修改标注：自适应转弯过滤】
        self.angular_z = 0.0
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # 【队友 2】只保留最新一帧 + BEST_EFFORT，避免 Camera 图像排队造成延迟。
        camera_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST
        )

        if USE_COMPRESSED:
            self.subscription = self.create_subscription(
                CompressedImage,
                COMPRESSED_TOPIC,
                self.listener_callback,
                camera_qos
            )
            waiting_topic = COMPRESSED_TOPIC
        else:
            self.subscription = self.create_subscription(
                Image,
                RAW_TOPIC,
                self.listener_callback,
                camera_qos
            )
            waiting_topic = RAW_TOPIC

        self.point_publisher = self.create_publisher(
            PointStamped,
            '/marker_position',
            10
        )

        print(
            f'[HSV DEBUG] see_marker started. Waiting for {waiting_topic} ...',
            flush=True
        )

    def odom_callback(self, msg):
        # 【中文修改标注：自适应转弯过滤】
        self.angular_z = float(msg.twist.twist.angular.z)

    def listener_callback(self, data):
        global _last_hsv_frame

        try:
            if USE_COMPRESSED:
                if len(data.data) == 0:
                    return
                current_frame = self.br.compressed_imgmsg_to_cv2(data)
            else:
                current_frame = decode_raw_image(data, self.br)
        except Exception as e:
            self.get_logger().error(f'Camera decode failed: {e}')
            return

        if not self._printed_camera_meta:
            if USE_COMPRESSED:
                print(
                    f'[CAMERA] compressed stream, '
                    f'frame_id={data.header.frame_id}, '
                    f'image_shape={current_frame.shape}',
                    flush=True
                )
            else:
                print(
                    f'[CAMERA] encoding={data.encoding}, width={data.width}, '
                    f'height={data.height}, step={data.step}, '
                    f'frame_id={data.header.frame_id}',
                    flush=True
                )
            self._printed_camera_meta = True

        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        _last_hsv_frame = hsv_frame

        combined_result = np.zeros_like(current_frame)

        # 每个合法 Marker 都包含 pink。
        pink_result, pink_blob = self.segment(
            current_frame,
            hsv_frame,
            'pink'
        )

        combined_result = cv2.bitwise_or(combined_result, pink_result)

        if pink_blob is not None:
            pink_x, pink_y, pink_h, pink_distance, pink_angle = pink_blob

            for colour in ['blue', 'green', 'yellow']:
                colour_result, colour_blob = self.segment(
                    current_frame,
                    hsv_frame,
                    colour
                )

                combined_result = cv2.bitwise_or(
                    combined_result,
                    colour_result
                )

                if colour_blob is None:
                    continue

                colour_x, colour_y, colour_h, colour_distance, colour_angle = colour_blob

                # 老师原本就有的横向配对检查。
                if abs(pink_x - colour_x) > pink_h:
                    continue

                # 【队友 2】再检查纵向距离，避免远处无关色块被配成 Marker。
                if abs(pink_y - colour_y) > pink_h:
                    continue

                # 【保留 (3)】上下两块尺寸应该大致相近。
                height_ratio = colour_h / pink_h
                if height_ratio < MIN_HEIGHT_RATIO or height_ratio > MAX_HEIGHT_RATIO:
                    continue

                # y 向下增加。
                # 【队友 2】使用位于 Marker 上半部的色块来计算 distance/angle。
                if colour_y < pink_y:
                    detected_type = colour + '/pink'
                    distance = colour_distance
                    angle = colour_angle
                else:
                    detected_type = 'pink/' + colour
                    distance = pink_distance
                    angle = pink_angle

                # 【重要】继续使用老师 landmark.py 的 marker_type 顺序。
                # 不采用队友文件自定义 MARKER_TYPES，避免 z/index 与 point_transformer/RViz 不一致。
                if detected_type not in marker_type:
                    self.get_logger().warning(
                        f'Unknown marker type: {detected_type}'
                    )
                    continue

                marker_index = marker_type.index(detected_type)
                marker_x, marker_y = self.polar_to_cartesian(distance, angle)

                marker_at = PointStamped()
                marker_at.header.stamp = self.get_clock().now().to_msg()
                marker_at.header.frame_id = 'camera_link'
                marker_at.point.x = float(marker_x)
                marker_at.point.y = float(marker_y)
                marker_at.point.z = float(marker_index)

                # 【中文修改标注：画面边缘质量过滤】
                image_width = current_frame.shape[1]
                edge_margin = image_width * EDGE_MARGIN_RATIO
                edge_safe = (
                    edge_margin <= pink_x <= image_width - edge_margin
                    and edge_margin <= colour_x <= image_width - edge_margin
                )

                if not edge_safe:
                    print(
                        f'[映射跳过-靠近画面边缘] '
                        f'pink_x={pink_x:.1f} | colour_x={colour_x:.1f}',
                        flush=True
                    )
                    continue

                # 【中文修改标注：自适应转弯过滤】
                abs_w = abs(self.angular_z)

                if abs_w > EMERGENCY_MAX_ANGULAR_SPEED:
                    print(
                        f'[映射跳过-转弯过快] '
                        f'angular_z={self.angular_z:.3f} rad/s',
                        flush=True
                    )
                    continue

                if abs_w > NORMAL_MAX_ANGULAR_SPEED:
                    shape_good = (
                        TURN_MIN_HEIGHT_RATIO
                        <= height_ratio
                        <= TURN_MAX_HEIGHT_RATIO
                    )
                    if not shape_good:
                        print(
                            f'[映射跳过-中等转弯且观测质量不足] '
                            f'angular_z={self.angular_z:.3f} rad/s | '
                            f'height_ratio={height_ratio:.2f}',
                            flush=True
                        )
                        continue

                    print(
                        f'[映射放行-自适应] '
                        f'中等转弯但观测质量较好 | '
                        f'angular_z={self.angular_z:.3f} rad/s | '
                        f'height_ratio={height_ratio:.2f}',
                        flush=True
                    )

                self.point_publisher.publish(marker_at)

                print(
                    f'[MARKER] {detected_type} | '
                    f'distance={distance:.3f} | angle={angle:.2f} deg | '
                    f'camera_xy=({marker_x:.3f}, {marker_y:.3f})',
                    flush=True
                )

                # 【队友 2】每一帧只发布一个有效 Marker candidate。
                break

        self.show_images(current_frame, combined_result)

    def segment(self, current_frame, hsv_frame, colour):
        lower, upper = COLOURS[colour]
        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)

        mask = cv2.inRange(hsv_frame, lower, upper)
        result = cv2.bitwise_and(current_frame, current_frame, mask=mask)

        blobs = cv2.connectedComponentsWithStats(
            mask,
            4,
            cv2.CV_32S
        )

        if SHOW_MASK_WINDOWS:
            cv2.imshow(colour, result)

        blob = self.get_stats(
            blobs,
            current_frame.shape[1],
            colour
        )

        return result, blob

    def get_stats(self, blobs, image_width, colour):
        num_labels, _, stats, centroids = blobs

        if num_labels <= 1:
            return None

        largest_area = 0
        rval = None
        centre = image_width / 2.0

        for i in range(1, num_labels):
            w = stats[i, cv2.CC_STAT_WIDTH]
            h = stats[i, cv2.CC_STAT_HEIGHT]
            area = stats[i, cv2.CC_STAT_AREA]
            cx, cy = centroids[i]

            if h < MIN_HEIGHT:
                continue

            if area < MIN_AREA:
                continue

            if area <= largest_area:
                continue

            largest_area = area

            # 【队友 2】2026-10-22 实机标定距离函数。
            # teammate note: 0.3-1.0 m, 8 points, residuals within ~2.3%.
            distance = 39.2644 * pow(h, -0.9413)

            # 【队友 2】用 w/h 判断横向被画面边缘裁切。
            aspect_ratio = w / h
            if aspect_ratio < 0.8:
                if cx < centre:
                    cx += h - w
                else:
                    cx -= h - w

            # 【队友 2】使用真实 image_width，且保留 signed bearing：
            # 左侧为正角，右侧为负角。
            angle = (
                (centre - cx)
                * FIELD_OF_VIEW_H
                / image_width
            )

            rval = (cx, cy, h, distance, angle)

        if rval is not None:
            cx, cy, h, distance, angle = rval
            print(
                f'[BLOB] {colour:<6} '
                f'cx={cx:.1f} cy={cy:.1f} h={h}px '
                f'area={largest_area} '
                f'distance={distance:.3f} angle={angle:.2f} deg',
                flush=True
            )

        return rval

    @staticmethod
    def polar_to_cartesian(distance, angle):
        angle_rad = math.radians(angle)

        # ==========================================================
        # 【中文修改标注：新几何模型】
        # 实验假设：
        # 由图像中 Marker 高度 h 拟合得到的 distance 更接近
        # 相机前向深度 x，而不是 LiDAR 式的斜距 R。
        #
        # ROS 坐标约定：
        #   +x = 前方
        #   +y = 左侧
        #
        # 因此：
        #   x = distance
        #   y = distance * tan(angle)
        #
        # 这个模型已经显式使用 angle 补偿斜视角，
        # 所以 2.x 分支不再要求 Marker 必须“接近正对”。
        # ==========================================================
        x = distance
        y = distance * math.tan(angle_rad)
        return x, y

    def show_images(self, current_frame, combined_result):
        cv2.imshow('camera', current_frame)
        cv2.setMouseCallback('camera', show_hsv)
        cv2.imshow('result', combined_result)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SeeMarker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
