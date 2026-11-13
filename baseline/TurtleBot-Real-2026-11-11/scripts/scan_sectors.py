"""Read-only live LaserScan bearings and recurring near-wall returns."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

SIDES = ('front', 'left', 'back', 'right')
STOP_RANGE = .25
WINDOW = 3.0


class SectorMonitor(Node):
    def __init__(self, output):
        super().__init__('physical_lidar_sector_observer')
        self.output = output
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.window = deque()
        self.sample = None
        self.received = None
        self.scans = 0
        self.create_subscription(LaserScan, '/scan', self.scan, qos_profile_sensor_data)
        self.create_timer(1.0, self.write)

    def scan(self, msg):
        now = time.monotonic()
        frame = msg.header.frame_id.lstrip('/')
        try:
            # The laser is rigidly attached; latest TF supplies its mounting orientation and offset.
            tf = self.tf.lookup_transform('base_footprint', frame, Time()).transform
            q, t = tf.rotation, tf.translation
            xx, xy = 1 - 2*(q.y*q.y+q.z*q.z), 2*(q.x*q.y-q.z*q.w)
            yx, yy = 2*(q.x*q.y+q.z*q.w), 1 - 2*(q.x*q.x+q.z*q.z)
            robot_tf = True
        except TransformException:
            robot_tf = False
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
        best, sides, bins = None, {}, {}
        valid = 0
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or not msg.range_min <= r <= msg.range_max:
                continue
            valid += 1
            a = msg.angle_min + i*msg.angle_increment
            scan_deg = math.remainder(math.degrees(a), 360)
            record = {'range_m': round(r, 4), 'scan_beam_deg': round(scan_deg, 2), 'index': i}
            if robot_tf:
                c, s = math.cos(a), math.sin(a)
                dx, dy = xx*c+xy*s, yx*c+yy*s
                heading = math.degrees(math.atan2(dy, dx))
                px, py = t.x + r*dx, t.y + r*dy
                side = SIDES[math.floor((heading+45)/90) % 4]
                sector = (math.floor((heading+15)/30) % 12) * 30
                record.update(beam_base_deg=round(heading, 2), side=side,
                              hit_base_xy_m=[round(px, 4), round(py, 4)],
                              hit_bearing_base_deg=round(math.degrees(math.atan2(py, px)), 2))
                sides[side] = min(r, sides.get(side, math.inf))
                bins[sector] = min(r, bins.get(sector, math.inf))
            if best is None or r < best[0]:
                best = (r, record)
        self.window.append((now, bins))
        self.sample = dict(stamp_unix=stamp, frame=frame, robot_transform_available=robot_tf,
                           total_beams=len(msg.ranges), valid_beams=valid, nearest=best[1] if best else None,
                           sector_min_range_m={side: round(sides[side], 4) if side in sides else None for side in SIDES})
        self.received = now
        self.scans += 1

    def write(self):
        now = time.monotonic()
        while self.window and now - self.window[0][0] > WINDOW:
            self.window.popleft()
        counts = {}
        for _, bins in self.window:
            for sector, distance in bins.items():
                hits, valid = counts.get(sector, (0, 0))
                counts[sector] = (hits + int(distance < STOP_RANGE), valid + 1)
        recurrent = []
        for sector, (hits, valid) in sorted(counts.items()):
            if hits:
                recurrent.append(dict(beam_center_base_deg=(sector+180)%360-180, half_width_deg=15,
                                      near_scans=hits, scans_with_returns=valid,
                                      fraction_all_scans=round(hits/len(self.window), 3)))
        age = None if self.received is None else now-self.received
        state = dict(updated_unix=time.time(), total_scans=self.scans, arrival_age_s=age,
                     scan_stamp_age_s=None if self.sample is None else time.time()-self.sample['stamp_unix'],
                     fresh=age is not None and age < .6,
                     angles='Beam directions in base_footprint: front=0, left=+90, back=±180, right=-90. '
                            'Quadrants are ±45 degrees. Range is measured from the lidar origin.',
                     threshold_m=STOP_RANGE, window_s=WINDOW, scans_in_window=len(self.window),
                     below_threshold_by_beam_sector=recurrent, latest=self.sample)
        tmp = self.output.with_suffix('.tmp')
        tmp.write_text(json.dumps(state, indent=2, allow_nan=False)+'\n')
        tmp.replace(self.output)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--duration', type=float, help='Stop after this many real seconds')
    args = p.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = SectorMonitor(args.output)
    try:
        if args.duration is None:
            rclpy.spin(node)
        else:
            deadline = time.monotonic() + args.duration
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=.05)
            node.write()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
