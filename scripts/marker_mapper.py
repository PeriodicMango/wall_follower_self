#!/usr/bin/env python3

import csv
import math
import rclpy
import tf2_ros
import tf2_geometry_msgs

from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray


# IMPORTANT:
# This ordering must be exactly the same as see_marker.py.
# /marker_position.point.z stores the index from this list.
MARKER_TYPES = [
    'blue/pink',
    'pink/blue',
    'green/pink',
    'pink/green',
    'yellow/pink',
    'pink/yellow'
]

# Detections of the same type within this distance are treated
# as repeated observations of the same physical marker.
MERGE_DISTANCE = 0.30

# RViz marker dimensions.
CYLINDER_DIAMETER = 0.12
HALF_HEIGHT = 0.10
TEXT_HEIGHT = 0.12


class MarkerMapper(Node):

    def __init__(self):
        super().__init__('marker_mapper')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer,
            self
        )

        self.point_subscriber = self.create_subscription(
            PointStamped,
            '/marker_position',
            self.point_callback,
            10
        )

        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/visualization_marker_array',
            10
        )

        # Each entry:
        # {
        #   'id': int,
        #   'type': str,
        #   'x': float,
        #   'y': float,
        #   'count': int
        # }
        self.landmarks = []
        self.next_landmark_id = 0

        self.get_logger().info(
            'MarkerMapper node has been initialised'
        )

    def point_callback(self, msg):
        # see_marker.py stores the marker type index in point.z.
        marker_index = int(round(msg.point.z))

        if marker_index < 0 or marker_index >= len(MARKER_TYPES):
            self.get_logger().warning(
                f'Invalid marker type index: {marker_index}'
            )
            return

        detected_type = MARKER_TYPES[marker_index]

        # The z field is not a real spatial coordinate.
        # Clear it before the TF transformation.
        point_for_tf = PointStamped()
        point_for_tf.header = msg.header
        point_for_tf.point.x = msg.point.x
        point_for_tf.point.y = msg.point.y
        point_for_tf.point.z = 0.0

        try:
            # Use the latest available transform, matching the provided
            # PointTransformer approach.
            transform = self.tf_buffer.lookup_transform(
                'map',
                msg.header.frame_id,
                rclpy.time.Time()
            )

            map_point = tf2_geometry_msgs.do_transform_point(
                point_for_tf,
                transform
            )

        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ) as e:
            self.get_logger().warning(
                f'Transform to map failed: {e}'
            )
            return

        map_x = map_point.point.x
        map_y = map_point.point.y

        landmark = self.find_existing_landmark(
            detected_type,
            map_x,
            map_y
        )

        if landmark is None:
            landmark = {
                'id': self.next_landmark_id,
                'type': detected_type,
                'x': map_x,
                'y': map_y,
                'count': 1
            }

            self.landmarks.append(landmark)
            self.next_landmark_id += 1

            self.get_logger().info(
                f'[NEW MARKER] type={detected_type} '
                f'map=({map_x:.3f}, {map_y:.3f})'
            )

        else:
            # Running average reduces jitter from repeated camera detections.
            count = landmark['count']

            landmark['x'] = (
                landmark['x'] * count + map_x
            ) / (count + 1)

            landmark['y'] = (
                landmark['y'] * count + map_y
            ) / (count + 1)

            landmark['count'] = count + 1

        self.publish_marker_array()

    def find_existing_landmark(
        self,
        detected_type,
        map_x,
        map_y
    ):
        for landmark in self.landmarks:
            if landmark['type'] != detected_type:
                continue

            distance = math.hypot(
                map_x - landmark['x'],
                map_y - landmark['y']
            )

            if distance < MERGE_DISTANCE:
                return landmark

        return None

    def publish_marker_array(self):
        marker_array = MarkerArray()

        for landmark in self.landmarks:
            top_colour, bottom_colour = landmark['type'].split('/')

            # Two cylinder halves are used so RViz shows both colours.
            bottom_marker = self.make_cylinder_marker(
                landmark,
                marker_id=landmark['id'] * 3,
                z=HALF_HEIGHT / 2.0,
                colour_name=bottom_colour
            )

            top_marker = self.make_cylinder_marker(
                landmark,
                marker_id=landmark['id'] * 3 + 1,
                z=HALF_HEIGHT + HALF_HEIGHT / 2.0,
                colour_name=top_colour
            )

            text_marker = self.make_text_marker(
                landmark,
                marker_id=landmark['id'] * 3 + 2,
                z=HALF_HEIGHT * 2.0 + 0.12
            )

            marker_array.markers.append(bottom_marker)
            marker_array.markers.append(top_marker)
            marker_array.markers.append(text_marker)

        self.marker_publisher.publish(marker_array)

    def make_cylinder_marker(
        self,
        landmark,
        marker_id,
        z,
        colour_name
    ):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'landmark_cylinders'
        marker.id = marker_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = landmark['x']
        marker.pose.position.y = landmark['y']
        marker.pose.position.z = z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = CYLINDER_DIAMETER
        marker.scale.y = CYLINDER_DIAMETER
        marker.scale.z = HALF_HEIGHT

        r, g, b = self.get_colour(colour_name)

        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0

        return marker

    def make_text_marker(
        self,
        landmark,
        marker_id,
        z
    ):
        marker = Marker()

        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = 'landmark_labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        marker.pose.position.x = landmark['x']
        marker.pose.position.y = landmark['y']
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0

        marker.scale.z = TEXT_HEIGHT

        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        marker.text = landmark['type']

        return marker

    def get_colour(self, colour_name):
        if colour_name == 'pink':
            return 1.0, 0.2, 0.6

        if colour_name == 'blue':
            return 0.0, 0.3, 1.0

        if colour_name == 'green':
            return 0.0, 1.0, 0.2

        if colour_name == 'yellow':
            return 1.0, 1.0, 0.0

        return 1.0, 1.0, 1.0

    def save_landmarks(self):
        try:
            with open(
                'landmarks.csv',
                'w',
                newline=''
            ) as csvfile:
                writer = csv.writer(csvfile)

                writer.writerow([
                    'id',
                    'type',
                    'x',
                    'y',
                    'observations'
                ])

                for landmark in self.landmarks:
                    writer.writerow([
                        landmark['id'],
                        landmark['type'],
                        landmark['x'],
                        landmark['y'],
                        landmark['count']
                    ])

            self.get_logger().info(
                f'Saved {len(self.landmarks)} landmarks to landmarks.csv'
            )

        except Exception as e:
            self.get_logger().error(
                f'Failed to save landmarks.csv: {e}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = MarkerMapper()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.save_landmarks()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
