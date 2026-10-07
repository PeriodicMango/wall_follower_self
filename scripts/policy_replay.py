#!/usr/bin/env python3
"""用录到的 bag 离线重放 handout 的策略，把「跑了什么」变成数字。

用法:
    python3 policy_replay.py <bag 目录> [更多 bag 目录...]

需要先 source ROS 环境。

关键设计: 所有统计只取「控制窗口」= [首条 /cmd_vel, 末条 /cmd_vel]。
    因为 /cmd_vel 只在控制器运行时存在, 窗口外就是控制器没跑、机器人静止的时段。
    不这样切, 静止段会把统计量彻底带偏 —— 静止的机器人停在某个位姿上,
    那个位姿的静态分支会被当成「控制器长时间停在某分支」。

输出:
    - 记录跨度、控制窗口、窗口外时长
    - RTF（sim 时间 / 墙钟），来自 /scan header 与 bag 时间戳
    - 每个分支: 帧数 / 帧占比 / 时间 / 进入次数 / 最长连续段
    - 指令积分 vs odom 实测（都在控制窗口内，换算到 sim 时间后给执行比例）
    - 近距样本数、切换序列

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


def min_in_arc(r, c, hw=BEAM_HW):
    n = len(r)
    best = float('inf')
    for i in range(c - hw, c + hw + 1):
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


def stamp_sec(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def replay(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))

    scans, odoms, cmds = [], [], []
    t0 = None
    pairs = {}
    cur_b = None
    cmd_by_branch = {b: 0 for b in range(1, 7)}

    while reader.has_next():
        topic, data, t = reader.read_next()
        if t0 is None:
            t0 = t
        if topic.endswith('/scan'):
            m = deserialize_message(data, LaserScan)
            r = m.ranges
            F, FL, LF, FR = min_in_arc(r, 0), min_in_arc(r, 30), min_in_arc(r, 60), min_in_arc(r, 330)
            b = classify(F, FL, LF, FR)
            cur_b = b
            scans.append((t, stamp_sec(m), b, min(F, FL, LF, FR)))
        elif topic.endswith('/odom'):
            o = deserialize_message(data, Odometry)
            p = o.pose.pose.position
            odoms.append((t, p.x, p.y, yaw_of(o.pose.pose.orientation)))
        elif topic.endswith('/cmd_vel'):
            tw = deserialize_message(data, Twist)
            cmds.append((t, tw.linear.x, tw.angular.z))
            k = '(%.2f,%+.2f)' % (tw.linear.x, tw.angular.z)
            pairs[k] = pairs.get(k, 0) + 1
            if cur_b is not None:
                cmd_by_branch[cur_b] += 1

    if not scans:
        print('=== %s\n  没有 /scan，无法分析\n' % path)
        return
    rec_span = (scans[-1][0] - t0) / NS

    # 控制窗口 = [首条非零速度指令, 末条非零速度指令]
    # 不能用「首末条 cmd_vel」: 场景脚本结束时自己会发一条零速停车命令,
    # near_start 触发时控制器也会先发一条 (0,0) 再退出 —— 两者都会把
    # 「控制器已经没在跑」的静止时段拉进窗口，而静止的机器人会被扫成
    # 某个静态分支（已踩过三次）。
    nz = [c for c in cmds if abs(c[1]) > 1e-9 or abs(c[2]) > 1e-9]
    if nz:
        w0, w1 = nz[0][0], nz[-1][0]
    elif cmds:
        w0, w1 = cmds[0][0], cmds[-1][0]
    else:
        w0, w1 = t0, scans[-1][0]
    win = (w1 - w0) / NS
    pre = max(0.0, (w0 - t0) / NS)
    post = max(0.0, (scans[-1][0] - w1) / NS)

    ws = [s for s in scans if w0 <= s[0] <= w1]
    wo = [o for o in odoms if w0 <= o[0] <= w1]
    wc = [c for c in cmds if w0 <= c[0] <= w1]

    # RTF: scan header 的 sim 时间跨度 / 墙钟跨度（窗口内）
    rtf = float('nan')
    if len(ws) > 1:
        sim_span = ws[-1][1] - ws[0][1]
        wall_span = (ws[-1][0] - ws[0][0]) / NS
        if wall_span > 0:
            rtf = sim_span / wall_span

    # 分支统计（窗口内）
    st = {b: {'frames': 0, 'time': 0.0, 'entries': 0,
              'run_max': 0.0, 'run_max_frames': 0, 'run_max_at': None}
          for b in range(1, 7)}
    seq = []
    prev = None
    last_t = None
    run_start = None
    run_frames = 0
    near = 0

    def close_run(b, end_t):
        if b is None or run_start is None:
            return
        dur = (end_t - run_start) / NS
        if dur > st[b]['run_max']:
            st[b]['run_max'] = dur
            st[b]['run_max_frames'] = run_frames
            st[b]['run_max_at'] = (run_start - w0) / NS

    for (t, s_sim, b, nearest) in ws:
        if nearest < NEAR:
            near += 1
        if b != prev:
            close_run(prev, t)
            run_start = t
            run_frames = 0
            st[b]['entries'] += 1
            seq.append((round((t - w0) / NS, 1), b))
        run_frames += 1
        st[b]['frames'] += 1
        if last_t is not None:
            st[b]['time'] += (t - last_t) / NS
        last_t = t
        prev = b
    close_run(prev, last_t)

    # 指令积分与 odom 实测（窗口内）
    cmd_d = cmd_a = 0.0
    for i in range(1, len(wc)):
        dt = (wc[i][0] - wc[i - 1][0]) / NS
        cmd_d += wc[i][1] * dt
        cmd_a += abs(wc[i][2]) * dt
    dist = yaw_abs = yaw_max = 0.0
    for i in range(1, len(wo)):
        dist += math.hypot(wo[i][1] - wo[i - 1][1], wo[i][2] - wo[i - 1][2])
        dy = wrap(wo[i][3] - wo[i - 1][3])
        yaw_abs += abs(dy)
        yaw_max = max(yaw_max, abs(dy))
    end_to_start = float('nan')
    if len(wo) > 1:
        end_to_start = math.hypot(wo[-1][1] - wo[0][1], wo[-1][2] - wo[0][2])

    tot_time = sum(st[b]['time'] for b in st) or 1.0
    print('=== %s' % path)
    print('  记录总跨度 %.1f s   控制窗口 %.1f s（窗口外: 前 %.1f s + 后 %.1f s，控制器未运行）'
          % (rec_span, win, pre, post))
    print('  窗口内扫描帧数 %d   实时因子 RTF %.2f（sim 时间 / 墙钟）  零速指令 %d 条（已排除在窗口外）'
          % (len(ws), rtf, len(cmds) - len(nz)))
    print()
    print('  分支  指令              帧数  帧占比   累计时间  时间占比  进入次数  最长连续')
    for b in range(1, 7):
        s = st[b]
        if s['frames'] == 0:
            continue
        at = ('@%.1fs' % s['run_max_at']) if s['run_max_at'] is not None else ''
        print('   %d   %-16s %5d  %5.1f%%   %6.1f s  %6.1f%%    %5d    %5.1f s%s'
              % (b, LEGEND[b], s['frames'], 100.0 * s['frames'] / max(len(ws), 1),
                 s['time'], 100.0 * s['time'] / tot_time, s['entries'], s['run_max'], at))
    print()
    champ = max(range(1, 7), key=lambda b: st[b]['run_max'])
    cs = st[champ]
    if cs['run_max'] > 0:
        print('  最长连续段（窗口内）: 分支%d 持续 %.1f s 墙钟（%.0f%% 的窗口，%d 帧，起于窗口内 %.1fs）'
              % (champ, cs['run_max'], 100.0 * cs['run_max'] / max(win, 1e-9),
                 cs['run_max_frames'], cs['run_max_at'] or 0.0))
    # 自检: 各分支的指令条数应与时间占比同比例；不一致就是有第二个发布者
    tot_cmd = sum(cmd_by_branch.values()) or 1
    print()
    print('  各分支的指令条数: %s'
          % '  '.join('%d:%d' % (b, cmd_by_branch[b]) for b in range(1, 7)
                      if cmd_by_branch[b] > 0))
    deltas = []
    for b in range(1, 7):
        if cmd_by_branch[b] > 0 and st[b]['frames'] > 0:
            mp = 100.0 * cmd_by_branch[b] / tot_cmd
            tp = 100.0 * st[b]['time'] / tot_time
            deltas.append((abs(mp - tp), b, mp, tp))
    if deltas:
        worst = max(deltas)
        flag = '  【可疑：疑似有第二个 cmd_vel 发布者】' if worst[0] > 15 else ''
        print('  自检: 逐分支指令占比 vs 时间占比，最大偏差 %.0f pt（分支%d: %.1f%% vs %.1f%%）%s'
              % (worst[0], worst[1], worst[2], worst[3], flag))
        print('        各分支偏差(pt): %s'
              % '  '.join('%d:%+.0f' % (b, mp - tp)
                          for _, b, mp, tp in sorted(deltas, key=lambda t: t[1])))
    print()
    if wc:
        vs = [c[1] for c in wc]
        aws = [abs(c[2]) for c in wc]
        print('  指令(cmd_vel) %d 条: 平均 linear %.3f m/s，平均|angular| %.3f rad/s'
              % (len(wc), sum(vs) / len(vs), sum(aws) / len(aws)))
        print('                  |linear| 最大 %.3f   |angular| 最大 %.3f' % (max(map(abs, vs)), max(aws)))
        top = sorted(pairs.items(), key=lambda kv: -kv[1])[:4]
        print('                  主要组合: %s' % '  '.join('%s x%d' % kv for kv in top))
    if len(wo) > 1 and rtf == rtf:
        print('  换算到 sim 时间后比较（RTF=%.2f）:' % rtf)
        print('    指令积分: 前进 %6.2f m   转角 %6.2f rad' % (cmd_d * rtf, cmd_a * rtf))
        print('    实际 odom: 前进 %6.2f m   转角 %6.2f rad（%.1f 圈）'
              % (dist, yaw_abs, yaw_abs / (2 * math.pi)))
        if cmd_d > 1e-6:
            print('    执行比例: 前进 %3.0f%%   转角 %3.0f%%'
                  % (100.0 * dist / (cmd_d * rtf), 100.0 * yaw_abs / max(cmd_a * rtf, 1e-6)))
        print('    odom 平均速度 %.3f m/s   平均|角速度| %.3f rad/s   单步最大|Δyaw| %.3f rad'
              % (dist / max(win, 1e-9), yaw_abs / max(win, 1e-9), yaw_max))
        print('    终点距起点 %.2f m' % end_to_start)
    print('  近距样本(<%.2f m): %d' % (NEAR, near))
    print('  切换序列(前 20，窗口内相对时刻): %s' % ['%d@%.1fs' % (b, tt) for tt, b in seq[:20]])
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
