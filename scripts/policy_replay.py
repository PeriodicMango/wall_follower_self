#!/usr/bin/env python3
"""用录到的 bag 离线重放 handout 的策略，把「跑了什么」变成数字。

用法:
    python3 policy_replay.py <bag 目录> [更多 bag 目录...]

需要先 source ROS 环境。

输出:
    - 扫描帧数、记录跨度、里程、终点距起点
    - 每个分支: 帧数 / 帧占比 / 累计时间 / 进入次数 / 最长连续段
    - 最长连续段（回答「有没有真的长时间停在某一分支」）
    - 指令 vs 实际: cmd_vel 的平均/最大线速度与角速度, odom 的实测速度与角速度
    - 近距样本数、切换序列

单位: rosbag2 的时间戳是纳秒, 本脚本内部一律先除以 1e9 再参与计算。

分支对照:
    1 = (0.2,+1.5) 左弧   2 = (0,-1.5) 原地右转   3 = (0.3,-1.5)
    4 = (0.3,+1.5)        5 = (0.3,+1.5)          6 = (0.3,0) 直行
"""

import math
import sys

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist
except ImportError as e:
    print('无法导入 ROS 依赖 (%s)。请先 source ROS 环境，例如:' % e)
    print('  source /opt/ros/humble/setup.bash')
    print('  source ~/turtlebot3_ws/install/setup.bash')
    sys.exit(1)

BEAM_HW = 15
NEAR = 0.16
NS = 1e9
LEGEND = {1: '(0.2,+1.5) 左弧', 2: '(0,-1.5) 原地右转', 3: '(0.3,-1.5)',
          4: '(0.3,+1.5)', 5: '(0.3,+1.5)', 6: '(0.3,0) 直行'}


def min_in_arc(r, center_deg, hw=BEAM_HW):
    n = len(r)
    best = float('inf')
    for i in range(center_deg - hw, center_deg + hw + 1):
        v = r[i % n]
        if v == v and v < best:
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


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def replay(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    st = {b: {'frames': 0, 'time': 0.0, 'entries': 0,
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
    yaw_prev, yaw_abs_total, yaw_max = None, 0.0, 0.0
    tfirst_odom = None

    cl_sum, ca_sum, cn = 0.0, 0.0, 0
    cl_max, ca_max = 0.0, 0.0
    pairs = {}

    def close_run(b, end_t):
        if b is None or run_start is None:
            return
        dur = (end_t - run_start) / NS
        if dur > st[b]['run_max']:
            st[b]['run_max'] = dur
            st[b]['run_max_frames'] = run_frames
            st[b]['run_max_at'] = (run_start - t0) / NS

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic.endswith('/scan'):
            r = deserialize_message(data, LaserScan).ranges
            F, FL, LF, FR = min_in_arc(r, 0), min_in_arc(r, 30), min_in_arc(r, 60), min_in_arc(r, 330)
            if min(F, FL, LF, FR) < NEAR:
                near += 1
            b = classify(F, FL, LF, FR)
            if b != prev:
                close_run(prev, t)
                run_start = t
                run_frames = 0
                st[b]['entries'] += 1
                seq.append((round((t - t0) / NS, 1), b))
            run_frames += 1
            st[b]['frames'] += 1
            if last_t is not None:
                st[b]['time'] += (t - last_t) / NS
            last_t = t
            nscan += 1
            prev = b
        elif topic.endswith('/odom'):
            o = deserialize_message(data, Odometry)
            p = o.pose.pose.position
            if start is None:
                start = (p.x, p.y)
                tfirst_odom = t
            if lastxy is not None:
                dist += math.hypot(p.x - lastxy[0], p.y - lastxy[1])
            lastxy = (p.x, p.y)
            y = yaw_of(o.pose.pose.orientation)
            if yaw_prev is not None:
                dy = wrap(y - yaw_prev)
                yaw_abs_total += abs(dy)
                if abs(dy) > yaw_max:
                    yaw_max = abs(dy)
            yaw_prev = y
        elif topic.endswith('/cmd_vel'):
            tw = deserialize_message(data, Twist)
            v, w = tw.linear.x, tw.angular.z
            cl_sum += v
            ca_sum += w
            cn += 1
            cl_max = max(cl_max, abs(v))
            ca_max = max(ca_max, abs(w))
            k = '(%.2f,%+.2f)' % (v, w)
            pairs[k] = pairs.get(k, 0) + 1

    if last_t is not None:
        close_run(prev, last_t)

    span = ((last_t - t0) / NS) if (last_t and t0) else 0.0
    tot_time = sum(st[b]['time'] for b in st) or 1.0
    print('=== %s' % path)
    print('  扫描帧数 %d   记录跨度 %.1f s   里程 %.2f m' % (nscan, span, dist))
    print()
    print('  分支  指令              帧数  帧占比   累计时间  时间占比  进入次数  最长连续')
    for b in range(1, 7):
        s = st[b]
        if s['frames'] == 0:
            continue
        at = ('@%.1fs' % s['run_max_at']) if s['run_max_at'] is not None else ''
        print('   %d   %-16s %5d  %5.1f%%   %6.1f s  %6.1f%%    %5d    %5.1f s%s'
              % (b, LEGEND[b], s['frames'], 100.0 * s['frames'] / max(nscan, 1),
                 s['time'], 100.0 * s['time'] / tot_time, s['entries'],
                 s['run_max'], at))
    print()
    champ = max(range(1, 7), key=lambda b: st[b]['run_max'])
    cs = st[champ]
    if cs['run_max'] > 0:
        print('  最长连续段: 分支%d 持续 %.1f s（%.1f%% 的跨度，%d 帧，起于 %.1fs）'
              % (champ, cs['run_max'], 100.0 * cs['run_max'] / max(span, 1e-9),
                 cs['run_max_frames'], cs['run_max_at'] or 0.0))
    print()
    if cn:
        print('  指令(cmd_vel): %d 条   平均 linear %.3f m/s  平均 angular %+.3f rad/s'
              % (cn, cl_sum / cn, ca_sum / cn))
        print('                 |linear| 最大 %.3f    |angular| 最大 %.3f'
              % (cl_max, ca_max))
        top = sorted(pairs.items(), key=lambda kv: -kv[1])[:4]
        print('                 主要组合: %s' % '  '.join('%s x%d' % (k, v) for k, v in top))
    if start and lastxy and tfirst_odom:
        ospan = (last_t - tfirst_odom) / NS
        print('  实际(odom): 平均速度 %.3f m/s   累计|转角| %.1f rad (%d 圈)'
              % (dist / max(ospan, 1e-9), yaw_abs_total, round(yaw_abs_total / (2 * math.pi))))
        print('              平均|角速度| %.3f rad/s   单步最大 |Δyaw| %.3f rad'
              % (yaw_abs_total / max(ospan, 1e-9), yaw_max))
        print('              终点距起点 %.2f m' % math.hypot(lastxy[0] - start[0], lastxy[1] - start[1]))
    print('  近距样本(<%.2f m): %d' % (NEAR, near))
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
