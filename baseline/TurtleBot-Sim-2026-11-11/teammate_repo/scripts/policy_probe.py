#!/usr/bin/env python3
"""对当前 /scan 套一遍 handout 的策略，打印四条束与命中的分支。

用法:
    python3 policy_probe.py                 # 打 3 帧
    python3 policy_probe.py --repeat 1      # 只打 1 帧
    python3 policy_probe.py --topic /scan

只订阅，不发布任何东西，不改任何状态。用来在某个 spawn 位姿下先看会走哪条分支，
不需要起控制器节点。

分支对照:
    1 = (0.2, +1.5) 左弧      2 = (0.0, -1.5) 原地右转
    3 = (0.3, -1.5)           4 = (0.3, +1.5)
    5 = (0.3, +1.5)           6 = (0.3,  0.0) 直行
"""

import argparse
import math
import sys

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
except ImportError as e:
    print('无法导入 ROS 依赖 (%s)。请先 source ROS 环境，例如:' % e)
    print('  source /opt/ros/humble/setup.bash')
    print('  source ~/turtlebot3_ws/install/setup.bash')
    sys.exit(1)

BEAM_HW = 15
LEGEND = {
    1: '(0.2, +1.5) 左弧',
    2: '(0.0, -1.5) 原地右转',
    3: '(0.3, -1.5)',
    4: '(0.3, +1.5)',
    5: '(0.3, +1.5)',
    6: '(0.3,  0.0) 直行',
}


def min_in_arc(r, center_deg, hw=BEAM_HW):
    """复刻 handout 的 scan_callback：在 ±hw 个索引内取最小值（索引按 1 度算）。"""
    n = len(r)
    best = float('inf')
    for i in range(center_deg - hw, center_deg + hw + 1):
        v = r[i % n]
        if v == v and v < best:      # v != v 用来剔除 NaN
            best = v
    return best


def classify(F, FL, LF, FR):
    if LF > 0.9:
        return 1
    if F < 0.7:
        return 2
    if FL < 0.6:
        return 3
    if FR < 0.6:
        return 4
    if LF > 0.6:
        return 5
    return 6


class Probe(Node):
    def __init__(self, topic, repeat):
        super().__init__('policy_probe')
        self.repeat = repeat
        self.count = 0
        self.create_subscription(LaserScan, topic, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        r = msg.ranges
        F = min_in_arc(r, 0)
        FL = min_in_arc(r, 30)
        LF = min_in_arc(r, 60)
        FR = min_in_arc(r, 330)
        b = classify(F, FL, LF, FR)

        n_inf = sum(1 for v in r if math.isinf(v))
        n_nan = sum(1 for v in r if v != v)
        n_zero = sum(1 for v in r if v == 0.0)
        self.count += 1

        print('[%d] points=%d  F=%.3f FL=%.3f LF=%.3f FR=%.3f  -> 分支%d %s'
              % (self.count, len(r), F, FL, LF, FR, b, LEGEND[b]), flush=True)
        print('     invalid: inf=%d nan=%d zero=%d   range_min=%.3f range_max=%.3f'
              % (n_inf, n_nan, n_zero, msg.range_min, msg.range_max), flush=True)

        if self.count >= self.repeat:
            rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/scan')
    ap.add_argument('--repeat', type=int, default=3)
    args = ap.parse_args()

    rclpy.init()
    node = Probe(args.topic, args.repeat)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
