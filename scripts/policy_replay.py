#!/usr/bin/env python3
"""用录到的 bag 离线重放 handout 的策略，把「跑了什么」变成数字。

用法:
    python3 policy_replay.py <bag 目录> [更多 bag 目录...]

需要先 source ROS 环境（为了 rclpy.serialization 与 rosbag2_py）。

输出每个 bag 的:
    - 扫描帧数、里程、终点距起点
    - 每个分支: 帧数 / 帧占比 / 累计时间 / 进入次数 / 最长连续段
    - 最长连续段总冠军（回答「有没有真的长时间停在某一分支」）
    - 近距样本数（任一束 < 0.16 m，视作贴墙/碰撞风险）
    - 切换序列（前 20 条）

为什么同时给「帧数」和「时间」: 时间按下滞时间累加，会把虚拟机卡顿摊进当时的分支；
帧数计数不受卡顿影响，两者对比即可看出卡顿占了多少。

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
LEGEND = {1: '(0.2,+1.5) 左弧', 2: '(0,-1.5) 原地右转', 3: '(0.3,-1.5)',
          4: '(0.3,+1.5)', 5: '(0.3,+1.5)', 6: '(0.3,0) 直行'}


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

    stat = {b: {'frames': 0, 'time': 0.0, 'entries': 0,
                'run_max': 0.0, 'run_max_frames': 0, 'run_max_at': None}
            for b in range(1, 7)}
    seq = []
    prev = None
    last_t = None
    t0 = None
    run_start = None
    run_frames = 0
    nscan, near = 0, 0
    dist, start, lastxy = 0.0, None, None

    def close_run(b, end_t):
        if b is None or run_start is None:
            return
        dur = end_t - run_start
        if dur > stat[b]['run_max']:
            stat[b]['run_max'] = dur
            stat[b]['run_max_frames'] = run_frames
            stat[b]['run_max_at'] = run_start - t0

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
            if b != prev:
                close_run(prev, t)
                run_start = t
                run_frames = 0
                stat[b]['entries'] += 1
                seq.append((round((t - t0) / 1e9, 1), b))
            run_frames += 1
            stat[b]['frames'] += 1
            if last_t is not None:
                stat[b]['time'] += (t - last_t) / 1e9
            last_t = t
            nscan += 1
            prev = b
        elif topic.endswith('/odom'):
            p = deserialize_message(data, Odometry).pose.pose.position
            if start is None:
                start = (p.x, p.y)
            if lastxy is not None:
                dist += math.hypot(p.x - lastxy[0], p.y - lastxy[1])
            lastxy = (p.x, p.y)

    if last_t is not None:
        close_run(prev, last_t)

    span = (last_t - t0) if (last_t and t0) else 0.0
    tot_time = sum(stat[b]['time'] for b in stat) or 1.0
    print('=== %s' % path)
    print('  扫描帧数 %d   记录跨度 %.1f s   里程 %.2f m' % (nscan, span, dist))
    print('  注: 帧占比不受卡顿影响；时间占比按下滞时间累加，会把卡顿摊进当时的分支')
    print()
    print('  分支  指令              帧数  帧占比   累计时间  时间占比  进入次数  最长连续')
    for b in range(1, 7):
        s = stat[b]
        if s['frames'] == 0:
            continue
        at = ('@%.1fs' % s['run_max_at']) if s['run_max_at'] is not None else ''
        print('   %d   %-16s %5d  %5.1f%%   %6.1f s  %6.1f%%    %5d    %5.1f s%s'
              % (b, LEGEND[b], s['frames'], 100.0 * s['frames'] / max(nscan, 1),
                 s['time'], 100.0 * s['time'] / tot_time, s['entries'],
                 s['run_max'], at))
    print()
    champ = max(range(1, 7), key=lambda b: stat[b]['run_max'])
    cs = stat[champ]
    if cs['run_max'] > 0:
        print('  最长连续段: 分支%d 持续 %.1f s（%.1f%% 的记录跨度，%d 帧，起于 %.1fs）'
              % (champ, cs['run_max'], 100.0 * cs['run_max'] / max(span, 1e-9),
                 cs['run_max_frames'],
                 cs['run_max_at'] if cs['run_max_at'] is not None else 0.0))
    print('  近距样本(<%.2f m): %d' % (NEAR, near))
    if start and lastxy:
        print('  终点距起点: %.2f m' % math.hypot(lastxy[0] - start[0], lastxy[1] - start[1]))
    print('  切换序列(前 20): %s' % ['%d@%.1fs' % (b, tt) for tt, b in seq[:20]])
    if len(seq) > 20:
        print('  切换总次数: %d（仅显示前 20）' % len(seq))
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
