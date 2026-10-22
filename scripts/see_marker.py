#!/usr/bin/env python3
import math
import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PointStamped

FIELD_OF_VIEW_H = 62.2

# IMPORTANT:
# point.z in /marker_position stores the index from this exact list.
# point_transformer.py must use the same ordering later.
MARKER_TYPES = [
    'blue/pink',
    'pink/blue',
    'green/pink',
    'pink/green',
    'yellow/pink',
    'pink/yellow'
]

# HSV ranges from the existing real-robot vision code.
COLOURS = {
    'pink':   ((150, 100, 90), (170, 170, 255)),
    'blue':   ((85, 140, 100), (115, 250, 255)),
    'green':  ((75, 190, 80), (85, 245, 150)),
    'yellow': ((20, 210, 170), (32, 255, 245))
}

MIN_HEIGHT = 3
MIN_AREA = 13


class SeeMarker(Node):

    def __init__(self):
        super().__init__('see_marker')

        self.br = CvBridge()

        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self.listener_callback,
            10
        )

        # Both wall_follower.cpp and point_transformer.py will subscribe
        # to this topic.
        self.point_publisher = self.create_publisher(
            PointStamped,
            '/marker_position',
            10
        )

        self.get_logger().info('SeeMarker node has been initialised')

    def listener_callback(self, data):
        if len(data.data) == 0:
            return

        try:
            current_frame = self.br.compressed_imgmsg_to_cv2(data)
        except Exception as e:
            self.get_logger().error(f'Camera decode failed: {e}')
            return

        hsv_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2HSV)
        combined_result = np.zeros_like(current_frame)

        # Every required marker contains pink.
        pink_result, pink_blob = self.segment(
            current_frame,
            hsv_frame,
            'pink'
        )

        if pink_blob is None:
            self.show_images(current_frame, combined_result)
            return

        combined_result = cv2.bitwise_or(
            combined_result,
            pink_result
        )

        pink_x, pink_y, pink_h, pink_distance, pink_angle = pink_blob

        # The second colour can be blue, green or yellow.
        for colour in ['blue', 'green', 'yellow']:
            colour_result, colour_blob = self.segment(
                current_frame,
                hsv_frame,
                colour
            )

            if colour_blob is None:
                continue

            combined_result = cv2.bitwise_or(
                combined_result,
                colour_result
            )

            colour_x, colour_y, colour_h, colour_distance, colour_angle = (
                colour_blob
            )

            # Keep the pairing logic from the existing vision code.
            # The two colour blobs should be close enough to belong to
            # the same cylindrical marker.
            if abs(pink_x - colour_x) > pink_h:
                continue

            if abs(pink_y - colour_y) > pink_h:
                continue

            # Image y increases downwards.
            if colour_y < pink_y:
                detected_type = colour + '/pink'
                distance = colour_distance
                angle = colour_angle
            else:
                detected_type = 'pink/' + colour
                distance = pink_distance
                angle = pink_angle

            if detected_type not in MARKER_TYPES:
                self.get_logger().warning(
                    f'Unknown marker type: {detected_type}'
                )
                continue

            marker_index = MARKER_TYPES.index(detected_type)

            marker_x, marker_y = self.polar_to_cartesian(
                distance,
                angle
            )

            marker_at = PointStamped()
            marker_at.header.stamp = self.get_clock().now().to_msg()
            marker_at.header.frame_id = 'camera_link'

            # Shared /marker_position contract:
            # x = forward distance in metres
            # y = lateral position: +left, -right
            # z = marker type index from MARKER_TYPES
            #
            # wall_follower.cpp uses x/y only.
            # point_transformer.py will read z first, set z=0, then
            # transform x/y into the map frame.
            marker_at.point.x = float(marker_x)
            marker_at.point.y = float(marker_y)
            marker_at.point.z = float(marker_index)

            self.point_publisher.publish(marker_at)

            self.get_logger().info(
                f'pink=(h={pink_h} x={pink_x:.0f} y={pink_y:.0f}) '
                f'other=(h={colour_h} x={colour_x:.0f} y={colour_y:.0f}) '
                f'distance={distance:.3f}m '
                f'angle={angle:.1f}deg '
                f'x={marker_x:.3f} y={marker_y:.3f}'
                # f'[MARKER] type={detected_type} '
                # f'index={marker_index} '
                # f'distance={distance:.3f}m '
                # f'angle={angle:.1f}deg '
                # f'x={marker_x:.3f} y={marker_y:.3f}'
            )

            # Process one marker candidate per frame for now.
            break

        self.show_images(current_frame, combined_result)

    def segment(self, current_frame, hsv_frame, colour):
        lower, upper = COLOURS[colour]

        lower = np.array(lower, dtype=np.uint8)
        upper = np.array(upper, dtype=np.uint8)

        mask = cv2.inRange(
            hsv_frame,
            lower,
            upper
        )

        result = cv2.bitwise_and(
            current_frame,
            current_frame,
            mask=mask
        )

        blobs = cv2.connectedComponentsWithStats(
            mask,
            4,
            cv2.CV_32S
        )

        blob = self.get_stats(
            blobs,
            current_frame.shape[1]
        )

        return result, blob

    def get_stats(self, blobs, image_width):
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

            # Distance model calibrated 2026-10-22 at 640x480:
            # 0.3-1.0 m in 0.1 m steps, 8 points, residuals within 2.3%.
            distance = 39.2644 * pow(h, -0.9413)

            # Existing correction for a partially visible marker near
            # the left or right image edge.
            aspect_ratio = w / h

            if aspect_ratio < 0.8:
                if cx < centre:
                    cx += h - w
                else:
                    cx -= h - w

            # Signed bearing:
            # positive = marker left of camera centre
            # negative = marker right of camera centre
            angle = (
                (centre - cx)
                * FIELD_OF_VIEW_H
                / image_width
            )

            rval = (
                cx,
                cy,
                h,
                distance,
                angle
            )

        return rval

    def polar_to_cartesian(self, distance, angle):
        angle_rad = math.radians(angle)

        # Matches wall_follower.cpp:
        # x > 0 = marker in front
        # y > 0 = marker on left
        # y < 0 = marker on right
        x = distance * math.cos(angle_rad)
        y = distance * math.sin(angle_rad)

        return x, y

    def show_images(self, current_frame, result):
        cv2.imshow('camera', current_frame)
        cv2.imshow('result', result)
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

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
