#!/usr/bin/env python3
"""用录到的 bag 离线重放 handout 的策略，把「跑了什么」变成数字。

用法:
    python3 policy_replay.py <bag 目录> [更多 bag 目录...]

需要先 source ROS 环境（为了 rclpy.serialization 与 rosbag2_py）。

输出每个 bag 的:
    - 扫描帧数、里程、终点距起点
    - 各分支占时比例（分支编号与 handout 的 if/else 一一对应）
    - 分支切换序列（前 20 条）
    - 近距样本数（任一束 < 0.16 m，视作贴墙/碰撞风险）

分支对照:
    1 = (0.2, +1.5) 左弧      2 = (0.0, -1.5) 原地右转
    3 = (0.3, -1.5)           4 = (0.3, +1.5)
    5 = (0.3, +1.5)           6 = (0.3,  0.0) 直行
"""

import math
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
except ImportError as e:
    print('无法导入 ROS 依赖 (%s)。请先 source ROS 环境，例如:' % e)
    print('  source /opt/ros/humble/setup.bash')
    print('  source ~/turtlebot3_ws/install/setup.bash')
    sys.exit(1)

BEAM_HW = 15        # handout 的 BEAM_WIDTH
NEAR = 0.16         # 近距阈值 (m)


def min_in_arc(r, center_deg, hw=BEAM_HW):
    n = len(r)
    best = float('inf')
    for i in range(center_deg - hw, center_deg + hw + 1):
        v = r[i % n]
        if v == v and v < best:      # 排除 NaN
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


def replay(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    bt = {i: 0.0 for i in range(1, 7)}
    seq, prev, last = [], None, None
    nscan, near = 0, 0
    dist, start, lastxy = 0.0, None, None
    t0 = None

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic.endswith('/scan'):
            r = deserialize_message(data, LaserScan).ranges
            F = min_in_arc(r, 0)
            FL = min_in_arc(r, 30)
            LF = min_in_arc(r, 60)
            FR = min_in_arc(r, 330)
            if min(F, FL, LF, FR) < NEAR:
                near += 1
            b = classify(F, FL, LF, FR)
            if last is not None:
                bt[b] += (t - last) / 1e9
            last = t
            nscan += 1
            if b != prev:
                seq.append((round((t - t0) / 1e9, 1), b))
                prev = b
        elif topic.endswith('/odom'):
            p = deserialize_message(data, Odometry).pose.pose.position
            if start is None:
                start = (p.x, p.y)
            if lastxy is not None:
                dist += math.hypot(p.x - lastxy[0], p.y - lastxy[1])
            lastxy = (p.x, p.y)

    total = sum(bt.values()) or 1.0
    print('=== %s' % path)
    print('  扫描帧数: %d   里程: %.2f m' % (nscan, dist))
    for b in sorted(bt):
        bar = '#' * int(round(40 * bt[b] / total))
        print('    分支%d  %7.1f s  %5.1f%%  %s' % (b, bt[b], 100 * bt[b] / total, bar))
    print('  近距样本(<%.2f m): %d' % (NEAR, near))
    if start and lastxy:
        print('  终点距起点: %.2f m' % math.hypot(lastxy[0] - start[0], lastxy[1] - start[1]))
    print('  切换序列(前 20): %s' % ['%d@%.1fs' % (b, tt) for tt, b in seq[:20]])
    print()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for p in sys.argv[1:]:
        try:
            replay(p)
        except Exception as e:
            print('=== %s\n  读取失败: %s\n' % (p, e))
    return 0


if __name__ == '__main__':
    sys.exit(main())
