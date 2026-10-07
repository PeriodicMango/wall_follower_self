#!/usr/bin/env python3
"""迷宫几何提取 + 场景位姿推荐（0.2 步骤 2/3 用）。

用法:
    python3 maze_geometry.py                      # 自动定位 turtlebot3_maze.world
    python3 maze_geometry.py <world.sdf>          # 指定文件
    python3 maze_geometry.py <world.sdf> <输出>   # 指定输出路径

输出: 终端摘要（末尾是候选位姿表）+ 完整结果写入文件（默认 ~/bags/maze_geometry.txt）

说明:
- 只读，不修改任何东西。
- 假定机器人 spawn 朝向 yaw=0：maze launch 只有 x_pose/y_pose，没有 yaw 参数，
  初始朝向由 world 里模型自身决定。所以位姿只需 (x, y)。
- 分支编号与 handout 一致：1=(0.2,+1.5) 左弧 / 2=(0.0,-1.5) 原地右转 /
  3=(0.3,-1.5) / 4=(0.3,+1.5) / 5=(0.3,+1.5) / 6=(0.3,0.0) 直行。
"""

import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

ROBOT_R = 0.22          # Waffle Pi 外接圆半径，用于判定「该处放得下机器人」
BEAM_HW = 15            # handout 的 BEAM_WIDTH
RANGE_MAX = 3.5         # 仿真 range_max
CELL = 0.15             # 地图格子边长 (m)
DEFAULT_OUT = os.path.expanduser('~/bags/maze_geometry.txt')


def find_world():
    try:
        prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'turtlebot3_gazebo'], text=True).strip()
        cand = os.path.join(prefix, 'share', 'turtlebot3_gazebo',
                            'worlds', 'turtlebot3_maze.world')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    return None


def get_pose(el, state_pose):
    p = state_pose if state_pose is not None else el.find('pose')
    if p is None or not p.text:
        return [0.0] * 6
    v = [float(x) for x in p.text.split()]
    while len(v) < 6:
        v.append(0.0)
    return v


def geoms_of(el):
    """返回 [(kind, *dims), ...]；先找 collision，没有再看 visual。

    注意: collision/visual 是 <link> 的子元素，不是 <model> 的直接子元素。
    只遍历本模型自己的 link，避免把嵌套子模型的几何也算进来。
    """
    links = el.findall('link')
    for tag in ('collision', 'visual'):
        out = []
        for link in links:
            for c in link.findall(tag):
                g = c.find('geometry')
                if g is None:
                    continue
                b = g.find('box')
                if b is not None:
                    s = b.find('size')
                    if s is not None and s.text:
                        x, y, z = [float(q) for q in s.text.split()]
                        out.append(('box', x, y, z))
                cy = g.find('cylinder')
                if cy is not None:
                    r, l = cy.find('radius'), cy.find('length')
                    if r is not None and l is not None and r.text and l.text:
                        out.append(('cyl', float(r.text), float(l.text)))
        if out:
            return out
    return []


def collect_models(el, px, py, pyaw, state, out, depth=0):
    for m in el.findall('model'):
        sp = state.get(m.get('name')) if depth == 0 else None
        v = get_pose(m, sp)
        wx = px + v[0] * math.cos(pyaw) - v[1] * math.sin(pyaw)
        wy = py + v[0] * math.sin(pyaw) + v[1] * math.cos(pyaw)
        wyaw = pyaw + v[5]
        out.append((m.get('name'), wx, wy, wyaw, geoms_of(m), depth,
                    not any(l.findall('collision') for l in m.findall('link'))))
        collect_models(m, wx, wy, wyaw, state, out, depth + 1)


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


def beams(boxes, circles, ox, oy):
    res = {}
    for name, ctr in (('F', 0), ('FL', 30), ('LF', 60), ('FR', 330)):
        best = RANGE_MAX
        for a in range(ctr - BEAM_HW, ctr + BEAM_HW + 1):
            rad = math.radians(a)
            dx, dy = math.cos(rad), math.sin(rad)
            d = RANGE_MAX
            for bx, by, byaw, sx, sy in boxes:
                t = ray_box(ox, oy, dx, dy, bx, by, byaw, sx, sy, d)
                if t is not None and t < d:
                    d = t
            for cx, cy, r in circles:
                t = ray_circle(ox, oy, dx, dy, cx, cy, r, d)
                if t is not None and t < d:
                    d = t
            if d < best:
                best = d
        res[name] = best
    return res


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


def point_free(boxes, circles, ox, oy):
    for bx, by, byaw, sx, sy in boxes:
        c, s = math.cos(-byaw), math.sin(-byaw)
        fx, fy = ox - bx, oy - by
        lx, ly = fx * c - fy * s, fx * s + fy * c
        ddx = max(abs(lx) - sx / 2.0, 0.0)
        ddy = max(abs(ly) - sy / 2.0, 0.0)
        if math.hypot(ddx, ddy) < ROBOT_R:
            return False
    for cx, cy, r in circles:
        if math.hypot(ox - cx, oy - cy) - r < ROBOT_R:
            return False
    return True


def main():
    world = sys.argv[1] if len(sys.argv) > 1 else find_world()
    if not world or not os.path.exists(world):
        print('找不到 world 文件。用法: python3 maze_geometry.py <world.sdf>')
        print('提示: ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds/')
        return 1
    out_path = sys.argv[2] if len(sys.argv) > 2 else DEFAULT_OUT

    root = ET.parse(world).getroot()
    w = root.find('world')
    if w is None:
        print('不是 world 文件:', world)
        return 1
    state = {}
    st = w.find('state')
    if st is not None:
        for m in st.findall('model'):
            p = m.find('pose')
            if p is not None:
                state[m.get('name')] = p

    models = []
    collect_models(w, 0.0, 0.0, 0.0, state, models)

    boxes, circles, markers = [], [], []
    lines = []
    lines.append('world: %s' % world)
    lines.append('模型总数: %d' % len(models))
    lines.append('')
    lines.append('%-34s %9s %9s %8s  %s' % ('name', 'x', 'y', 'yaw', 'geometry'))
    for name, x, y, yaw, gs, depth, visual_only in models:
        gs_txt = ' '.join('%s(%s)' % (g[0], ','.join('%.3f' % q for q in g[1:]))
                          for g in gs) or '-'
        if gs and visual_only:
            gs_txt += '  [无 collision，取 visual]'
        lines.append('%-34s %9.3f %9.3f %8.3f  %s'
                     % ('  ' * depth + name, x, y, yaw, gs_txt))
        for g in gs:
            if g[0] == 'box':
                boxes.append((x, y, yaw, g[1], g[2]))
            else:
                if (x, y, g[1]) not in circles:
                    circles.append((x, y, g[1]))
                # marker 模型常有两个半圆柱（上/下），只记一次
                if 'marker' in name and not any(mk[0] == name for mk in markers):
                    markers.append((name, x, y, g[1], g[2]))
    lines.append('')
    lines.append('障碍: %d 个 box, %d 个圆柱' % (len(boxes), len(circles)))
    lines.append('')
    lines.append('marker:')
    for name, x, y, r, l in markers:
        lines.append('  %-28s x=%8.3f y=%8.3f r=%.3f h=%.3f' % (name, x, y, r, l))
    lines.append('')

    if not boxes:
        print('\n'.join(lines[-6:]))
        print('警告: 没解析到任何 box 障碍，无法生成地图。')
        return 1

    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    x0, x1 = min(xs) - 0.6, max(xs) + 0.6
    y0, y1 = min(ys) - 0.6, max(ys) + 0.6
    nx = int((x1 - x0) / CELL) + 1
    ny = int((y1 - y0) / CELL) + 1

    grid, cells = [], []
    for j in range(ny - 1, -1, -1):
        row = []
        for i in range(nx):
            x, y = x0 + i * CELL, y0 + j * CELL
            if not point_free(boxes, circles, x, y):
                row.append('#')
                continue
            b = beams(boxes, circles, x, y)
            br = classify(b['F'], b['FL'], b['LF'], b['FR'])
            row.append(str(br))
            cells.append((x, y, b, br))
        grid.append(''.join(row))

    lines.append('地图 (%.2f m/格; 数字=该位姿(朝向 yaw=0)会命中的分支, #=机器人放不下)' % CELL)
    lines.append('y 从下到上递增, x 从左到右递增; 左下角 = (%.2f, %.2f)' % (x0, y0))
    for k, row in enumerate(grid):
        lines.append('  ' + row)
    lines.append('')

    def fmt(x, y, b, br, tag):
        return ('  %-6s x=%7.3f y=%7.3f  F=%5.2f FL=%5.2f LF=%5.2f FR=%5.2f  分支%d  %s'
                % (tag, x, y, b['F'], b['FL'], b['LF'], b['FR'], br, ''))

    picks = []
    # A 基线: 就是默认 spawn (0,0)，直接算，不做吸附
    ba = beams(boxes, circles, 0.0, 0.0)
    br_a = classify(ba['F'], ba['FL'], ba['LF'], ba['FR'])
    free_a = point_free(boxes, circles, 0.0, 0.0)
    picks.append(('A基线', (0.0, 0.0, ba, br_a)))
    if not free_a:
        lines.append('警告: (0, 0) 处机器人放不下（在墙内或距墙 < %.2f m），默认 spawn 会卡住'
                     % ROBOT_R)
    # C 开阔区: 四束最小值最大的格
    if cells:
        picks.append(('C开阔', max(cells, key=lambda c: min(c[2].values()))))
    # D 贴墙: 前方通畅、左侧 0.15~0.30 m
    d = [c for c in cells if c[2]['F'] > 0.7 and 0.15 <= c[2]['LF'] <= 0.30]
    if d:
        picks.append(('D贴墙', min(d, key=lambda c: abs(c[2]['LF'] - 0.2))))
    # B 正对墙: 必须真的命中分支 2（前方 < 0.7 且左前 <= 0.9，否则会被分支 1 抢先）
    b2 = [c for c in cells if c[3] == 2]
    if b2:
        picks.append(('B对墙', max(b2, key=lambda c: c[2]['F'])))

    lines.append('候选位姿（launch 参数 x_pose / y_pose；朝向由 world 决定，无法用 launch 指定）:')
    for tag, c in picks:
        x, y, b, br = c
        note = '' if point_free(boxes, circles, x, y) else '  [该处放不下机器人!]'
        lines.append('  %-6s x_pose:=%.2f y_pose:=%.2f   F=%.2f FL=%.2f LF=%.2f FR=%.2f -> 分支%d%s'
                     % (tag, x, y, b['F'], b['FL'], b['LF'], b['FR'], br, note))

    text = '\n'.join(lines) + '\n'
    try:
        os.makedirs(os.path.dirname(out_path), exist_ok=True)
        with open(out_path, 'w') as f:
            f.write(text)
    except Exception as e:
        print('写文件失败:', e)

    # 终端只打摘要，候选表放最后
    print('world: %s   模型 %d   障碍 %d box / %d 圆柱   地图 %dx%d'
          % (os.path.basename(world), len(models), len(boxes), len(circles), nx, ny))
    print('完整结果已写入: %s' % out_path)
    print()
    for name, x, y, r, l in markers:
        print('  marker %-26s x=%8.3f y=%8.3f r=%.3f' % (name, x, y, r))
    print()
    print('候选位姿（复制到 launch 命令里）:')
    for tag, c in picks:
        x, y, b, br = c
        note = '' if point_free(boxes, circles, x, y) else '  [该处放不下机器人!]'
        print('  %-6s x_pose:=%.2f y_pose:=%.2f   F=%.2f FL=%.2f LF=%.2f FR=%.2f -> 分支%d%s'
              % (tag, x, y, b['F'], b['FL'], b['LF'], b['FR'], br, note))
    print()
    print('地图与几何明细见 %s（可用 cat 分段查看）' % out_path)
    return 0


if __name__ == '__main__':
    sys.exit(main())
