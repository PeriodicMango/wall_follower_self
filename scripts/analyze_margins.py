#!/usr/bin/env python3
"""按 maze_geometry.txt 的几何，扫描整个场地的分支稳定性（0.2/0.3 用）。

用法:
    python3 analyze_margins.py [maze_geometry.txt] [--lds-dx -0.06] [--step 0.10] [--margin 0.10]

所谓「稳定」：该点周围的 8 个邻点（±margin）命中同一个分支。
输出各分支的稳定格点数与每分支一个最优代表位姿。

--lds-dx 是把 spawn 坐标换算到「等效 LDS 位置」的标定常数：
实测 spawn 于 (2.90, 1.18) 时，/scan 与模型在 (2.84, 1.18) 处一致 → lds-dx = -0.06。
"""

import argparse
import math
import re
import sys
from collections import Counter

BEAM_HW = 15
RANGE_MAX = 3.5
ROBOT_R = 0.22


def load(path):
    walls, marks = [], []
    for line in open(path):
        m = re.match(r"\s*(\S+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(\S+)\s+(.*)$", line)
        if not m:
            continue
        name, x, y, yaw, geom = m.group(1), float(m.group(2)), float(m.group(3)), float(m.group(4)), m.group(6)
        if 'box(' in geom:
            walls.append((x, y, yaw, 1.0, 0.01))
        elif 'cyl(' in geom and 'marker' in name:
            marks.append((x, y, float(geom.split('cyl(')[1].split(',')[0])))
    return walls, marks


def ray_box(ox, oy, dx, dy, bx, by, byaw, sx, sy, tmax):
    c, s = math.cos(-byaw), math.sin(-byaw)
    fx, fy = ox - bx, oy - by
    lx, ly = fx * c - fy * s, fx * s + fy * c
    ldx, ldy = dx * c - dy * s, dx * s + dy * c
    hx, hy = sx / 2.0, sy / 2.0
    tmin, tmax2 = 0.0, tmax
    for p, d, h in ((lx, ldx, hx), (ly, ldy, hy)):
        if abs(d) < 1e-9:
            if p < -h or p > h:
                return None
        else:
            t1, t2 = (-h - p) / d, (h - p) / d
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax2 = min(tmax2, t2)
            if tmin > tmax2:
                return None
    return tmin if tmin > 1e-6 else None


def ray_circle(ox, oy, dx, dy, cx, cy, r, tmax):
    fx, fy = ox - cx, oy - cy
    b = 2.0 * (fx * dx + fy * dy)
    c = fx * fx + fy * fy - r * r
    disc = b * b - 4.0 * c
    if disc < 0.0:
        return None
    sq = math.sqrt(disc)
    for t in ((-b - sq) / 2.0, (-b + sq) / 2.0):
        if 1e-6 < t <= tmax:
            return t
    return None


def beams(walls, marks, ox, oy):
    out = {}
    for name, ctr in (('F', 0), ('FL', 30), ('LF', 60), ('FR', 330)):
        best = RANGE_MAX
        for a in range(ctr - BEAM_HW, ctr + BEAM_HW + 1):
            rad = math.radians(a)
            dx, dy = math.cos(rad), math.sin(rad)
            d = RANGE_MAX
            for bx, by, byaw, sx, sy in walls:
                t = ray_box(ox, oy, dx, dy, bx, by, byaw, sx, sy, d)
                if t is not None and t < d:
                    d = t
            for cx, cy, r in marks:
                t = ray_circle(ox, oy, dx, dy, cx, cy, r, d)
                if t is not None and t < d:
                    d = t
            if d < best:
                best = d
        out[name] = best
    return out


def classify(b):
    if b['LF'] > 0.9:
        return 1
    if b['F'] < 0.7:
        return 2
    if b['FL'] < 0.6:
        return 3
    if b['FR'] < 0.6:
        return 4
    if b['LF'] > 0.6:
        return 5
    return 6


def free(walls, marks, ox, oy):
    for bx, by, byaw, sx, sy in walls:
        c, s = math.cos(-byaw), math.sin(-byaw)
        fx, fy = ox - bx, oy - by
        lx, ly = fx * c - fy * s, fx * s + fy * c
        if math.hypot(max(abs(lx) - sx / 2, 0), max(abs(ly) - sy / 2, 0)) < ROBOT_R:
            return False
    for cx, cy, r in marks:
        if math.hypot(ox - cx, oy - cy) - r < ROBOT_R:
            return False
    return True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('geometry', nargs='?',
                    default='~/wall_follower_artifacts/vm/maze_geometry.txt')
    ap.add_argument('--lds-dx', type=float, default=-0.06)
    ap.add_argument('--step', type=float, default=0.10)
    ap.add_argument('--margin', type=float, default=0.10)
    ap.add_argument('--x0', type=float, default=0.10)
    ap.add_argument('--x1', type=float, default=4.00)
    ap.add_argument('--y0', type=float, default=0.05)
    ap.add_argument('--y1', type=float, default=3.95)
    a = ap.parse_args()

    import os
    path = os.path.expanduser(a.geometry)
    walls, marks = load(path)
    print('几何: %s   墙 %d 面, marker %d 个' % (path, len(walls), len(marks)))
    print('LDS 偏移标定 lds_dx=%.2f m  网格 %.2f m  稳定性判据 ±%.2f m'
          % (a.lds_dx, a.step, a.margin))

    xs = [round(a.x0 + i * a.step, 3) for i in range(int((a.x1 - a.x0) / a.step) + 1)]
    ys = [round(a.y0 + j * a.step, 3) for j in range(int((a.y1 - a.y0) / a.step) + 1)]
    grid = {}
    for x in xs:
        for y in ys:
            if free(walls, marks, x, y):
                b = beams(walls, marks, x + a.lds_dx, y)
                grid[(x, y)] = (b, classify(b))

    nb = [(a.step, 0), (-a.step, 0), (0, a.step), (0, -a.step),
          (a.step, a.step), (a.step, -a.step), (-a.step, a.step), (-a.step, -a.step)]
    stable = []
    for (x, y), (b, c) in grid.items():
        if all(grid.get((round(x + dx, 3), round(y + dy, 3)), (None, None))[1] == c
               for dx, dy in nb):
            stable.append((x, y, b, c))

    n, s = len(grid), len(stable)
    print()
    print('可放机器人的格点 %d，其中 ±%.2f m 内分支不变的稳定格点 %d (%.0f%%)'
          % (n, a.margin, s, 100.0 * s / max(n, 1)))
    print('稳定格点分支分布:', dict(sorted(Counter(c for _, _, _, c in stable).items())))
    print()
    print('稳定候选（每分支取「四束最小值最大」的一个）:')
    for c in (1, 2, 3, 4, 5, 6):
        sub = [t for t in stable if t[3] == c]
        if not sub:
            print('  分支%d  —— 无稳定点' % c)
            continue
        x, y, b, _ = max(sub, key=lambda t: min(t[2].values()))
        print('  分支%d  x_pose:=%.2f y_pose:=%.2f   F=%.2f FL=%.2f LF=%.2f FR=%.2f'
              % (c, x, y, b['F'], b['FL'], b['LF'], b['FR']))
    return 0


if __name__ == '__main__':
    sys.exit(main())
