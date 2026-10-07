#!/usr/bin/env python3
"""迷宫几何提取 + 场景位姿推荐（0.2 步骤 2/3 用）。

用法:
    python3 maze_geometry.py                            # 自动定位 turtlebot3_maze.world
    python3 maze_geometry.py <world.sdf>                # 指定文件
    python3 maze_geometry.py <world.sdf> --out <文件>    # 指定输出
    python3 maze_geometry.py <world.sdf> --models <目录> # 追加 model:// 搜索目录（可重复）

会跟随 <include><uri>model://NAME</uri></include>，到 NAME/model.sdf 取几何。
搜索顺序: $GAZEBO_MODEL_PATH 各段 → <turtlebot3_gazebo>/share/.../models → ~/.gazebo/models → --models。

说明:
- 只读，不修改任何东西。
- 假定机器人 spawn 朝向 yaw=0：maze launch 只有 x_pose/y_pose，没有 yaw 参数。
- 分支编号与 handout 一致：1=(0.2,+1.5) 左弧 / 2=(0.0,-1.5) 原地右转 /
  3=(0.3,-1.5) / 4=(0.3,+1.5) / 5=(0.3,+1.5) / 6=(0.3,0.0) 直行。
"""

import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

ROBOT_R = 0.22          # Waffle Pi 外接圆半径
BEAM_HW = 15            # handout 的 BEAM_WIDTH
RANGE_MAX = 3.5         # 仿真 range_max
CELL = 0.15             # 地图格子边长 (m)
DEFAULT_OUT = os.path.expanduser('~/bags/maze_geometry.txt')


# ----------------------------------------------------------------- 工具

def txt(el, tag):
    c = el.find(tag)
    return c.text.strip() if c is not None and c.text else ''


def parse_pose(el):
    if el is None or not el.text:
        return [0.0] * 6
    v = [float(x) for x in el.text.split()]
    while len(v) < 6:
        v.append(0.0)
    return v


def compose(px, py, pyaw, v):
    return (px + v[0] * math.cos(pyaw) - v[1] * math.sin(pyaw),
            py + v[0] * math.sin(pyaw) + v[1] * math.cos(pyaw),
            pyaw + v[5])


def geoms_of(model):
    """返回 [(kind, *dims), ...]；先 collision，没有再取 visual。

    collision/visual 是 <link> 的子元素，不是 <model> 的，只遍历本模型的 link。
    """
    links = model.findall('link')
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
                me = g.find('mesh')
                if me is not None:
                    out.append(('mesh', 0.0, 0.0, 0.0))
        if out:
            return out
    return []


def has_collision(model):
    return any(l.findall('collision') for l in model.findall('link'))


# ----------------------------------------------------------------- model:// 解析

def build_search_dirs(extra):
    dirs = []
    for p in os.environ.get('GAZEBO_MODEL_PATH', '').split(':'):
        if p:
            dirs.append(p)
    try:
        prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'turtlebot3_gazebo'],
            text=True, stderr=subprocess.DEVNULL).strip()
        dirs.append(os.path.join(prefix, 'share', 'turtlebot3_gazebo', 'models'))
    except Exception:
        pass
    import glob
    dirs.extend(sorted(glob.glob('/usr/share/gazebo-*/models')))
    dirs.append(os.path.expanduser('~/.gazebo/models'))
    dirs.extend(extra)
    return [d for d in dirs if os.path.isdir(d)]


def resolve_model(uri, search_dirs):
    if not uri.startswith('model://'):
        return None
    name = uri[len('model://'):].strip().strip('/')
    for d in search_dirs:
        for cand in (os.path.join(d, name, 'model.sdf'),
                     os.path.join(d, name + '.sdf')):
            if os.path.exists(cand):
                return cand
    return None


# ----------------------------------------------------------------- 遍历

def walk(node, px, py, pyaw, state, out, search_dirs, unresolved, depth=0):
    for child in list(node):
        if child.tag == 'model':
            sp = state.get(child.get('name')) if depth == 0 else None
            v = parse_pose(sp if sp is not None else child.find('pose'))
            wx, wy, wyaw = compose(px, py, pyaw, v)
            out.append((child.get('name'), wx, wy, wyaw, geoms_of(child),
                        depth, not has_collision(child), 'model'))
            walk(child, wx, wy, wyaw, state, out, search_dirs, unresolved, depth + 1)
        elif child.tag == 'include':
            uri = txt(child, 'uri')
            v = parse_pose(child.find('pose'))
            wx, wy, wyaw = compose(px, py, pyaw, v)
            path = resolve_model(uri, search_dirs)
            if path is None:
                # ground_plane / sun 本来就没有可当作障碍的几何，不算错误
                if uri.rstrip('/').rsplit('/', 1)[-1] not in ('ground_plane', 'sun'):
                    unresolved.append((uri, wx, wy))
                out.append((uri or '<include 无 uri>', wx, wy, wyaw, [],
                            depth, False, 'include?!'))
                continue
            try:
                root = ET.parse(path).getroot()
            except Exception as e:
                unresolved.append(('%s (%s)' % (uri, e), wx, wy))
                out.append((uri, wx, wy, wyaw, [], depth, False, 'include?!'))
                continue
            m = root.find('model')
            if m is None:
                unresolved.append(('%s (no <model> in %s)' % (uri, path), wx, wy))
                continue
            mv = parse_pose(m.find('pose'))
            wx2, wy2, wyaw2 = compose(wx, wy, wyaw, mv)
            name = m.get('name') or uri
            out.append((name, wx2, wy2, wyaw2, geoms_of(m), depth,
                        not has_collision(m), 'include'))
            walk(m, wx2, wy2, wyaw2, state, out, search_dirs, unresolved, depth + 1)


# ----------------------------------------------------------------- 几何运算

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


# ----------------------------------------------------------------- main

def find_world():
    try:
        prefix = subprocess.check_output(
            ['ros2', 'pkg', 'prefix', 'turtlebot3_gazebo'],
            text=True, stderr=subprocess.DEVNULL).strip()
        cand = os.path.join(prefix, 'share', 'turtlebot3_gazebo',
                            'worlds', 'turtlebot3_maze.world')
        if os.path.exists(cand):
            return cand
    except Exception:
        pass
    return None


def main():
    argv = sys.argv[1:]
    extra_models, out_path, world = [], DEFAULT_OUT, None
    i = 0
    while i < len(argv):
        if argv[i] == '--models' and i + 1 < len(argv):
            extra_models.append(argv[i + 1]); i += 2
        elif argv[i] == '--out' and i + 1 < len(argv):
            out_path = argv[i + 1]; i += 2
        else:
            world = argv[i]; i += 1
    if world is None:
        world = find_world()
    if not world or not os.path.exists(world):
        print('找不到 world 文件。用法: python3 maze_geometry.py <world.sdf>')
        print('提示: ls $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds/')
        return 1

    search_dirs = build_search_dirs(extra_models)
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

    models, unresolved = [], []
    walk(w, 0.0, 0.0, 0.0, state, models, search_dirs, unresolved)

    boxes, circles, markers, meshes = [], [], [], []
    lines = ['world: %s' % world, 'model:// 搜索目录: %s' % (search_dirs or '(无)'),
             '模型条目: %d   未解析的 include: %d' % (len(models), len(unresolved)), '']
    lines.append('%-34s %9s %9s %8s  %-7s %s'
                 % ('name', 'x', 'y', 'yaw', '来源', 'geometry'))
    for name, x, y, yaw, gs, depth, no_coll, src in models:
        gs_txt = ' '.join('%s(%s)' % (g[0], ','.join('%.3f' % q for q in g[1:]))
                          for g in gs) or '-'
        if gs and no_coll:
            gs_txt += '  [无 collision，取 visual]'
        lines.append('%-34s %9.3f %9.3f %8.3f  %-7s %s'
                     % ('  ' * depth + str(name), x, y, yaw, src, gs_txt))
        for g in gs:
            if g[0] == 'box':
                boxes.append((x, y, yaw, g[1], g[2]))
            elif g[0] == 'cyl':
                if (x, y, g[1]) not in circles:
                    circles.append((x, y, g[1]))
                if 'marker' in str(name) and not any(mk[0] == name for mk in markers):
                    markers.append((name, x, y, g[1], g[2]))
            else:
                meshes.append((name, x, y))
    lines.append('')
    lines.append('障碍: %d box, %d 圆柱, %d mesh（mesh 无法做射线投射，会漏掉）'
                 % (len(boxes), len(circles), len(meshes)))
    if unresolved:
        lines.append('')
        lines.append('未解析的 include（这些部分的几何缺失，位姿会算错）:')
        for uri, x, y in unresolved:
            lines.append('  %s  @ (%.3f, %.3f)' % (uri, x, y))
    lines.append('')
    lines.append('marker:')
    for name, x, y, r, l in markers:
        lines.append('  %-28s x=%8.3f y=%8.3f r=%.3f h=%.3f' % (name, x, y, r, l))

    # ---------------- 摘要（终端）
    print('world: %s' % os.path.basename(world))
    print('model:// 搜索目录: %s' % (', '.join(search_dirs) or '(无)'))
    print('模型条目 %d   障碍 %d box / %d 圆柱 / %d mesh   未解析 include %d'
          % (len(models), len(boxes), len(circles), len(meshes), len(unresolved)))
    if unresolved:
        print('  未解析（几何会缺失）:')
        for uri, x, y in unresolved:
            print('    %s' % uri)
    if meshes:
        print('  含 mesh 的模型（本脚本不解析 mesh）:')
        for name, x, y in meshes[:5]:
            print('    %s @ (%.2f, %.2f)' % (name, x, y))
    print()
    for name, x, y, r, l in markers:
        print('  marker %-26s x=%8.3f y=%8.3f r=%.3f' % (name, x, y, r))

    if not boxes:
        text = '\n'.join(lines) + '\n'
        try:
            os.makedirs(os.path.dirname(out_path), exist_ok=True)
            with open(out_path, 'w') as f:
                f.write(text)
        except Exception as e:
            print('写文件失败:', e)
        print()
        print('没有任何 box 障碍，无法生成地图。明细见 %s' % out_path)
        return 1

    xs = [b[0] for b in boxes]
    ys = [b[1] for b in boxes]
    x0, x1 = min(xs) - 0.6, max(xs) + 0.6
    y0, y1 = min(ys) - 0.6, max(ys) + 0.6
    nx = int((x1 - x0) / CELL) + 1
    ny = int((y1 - y0) / CELL) + 1

    # 每个格子的状态
    free_at, at = set(), {}
    for j in range(ny):
        for i in range(nx):
            x, y = x0 + i * CELL, y0 + j * CELL
            if not point_free(boxes, circles, x, y):
                continue
            free_at.add((i, j))
            b = beams(boxes, circles, x, y)
            at[(i, j)] = (x, y, b, classify(b['F'], b['FL'], b['LF'], b['FR']))

    # 只保留与迷宫中心连通的自由格，否则外墙以外的空旷地会被选成候选
    ccx, ccy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    inside = set()
    if free_at:
        seed = min(free_at, key=lambda c: (x0 + c[0] * CELL - ccx) ** 2
                                            + (y0 + c[1] * CELL - ccy) ** 2)
        stack = [seed]
        inside.add(seed)
        while stack:
            i, j = stack.pop()
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nb = (i + di, j + dj)
                if nb in free_at and nb not in inside:
                    inside.add(nb)
                    stack.append(nb)

    grid, cells = [], []
    for j in range(ny - 1, -1, -1):
        row = []
        for i in range(nx):
            if (i, j) not in free_at:
                row.append('#')
            elif (i, j) not in inside:
                row.append('.')
            else:
                row.append(str(at[(i, j)][3]))
                cells.append(at[(i, j)])
        grid.append(''.join(row))

    lines.append('')
    lines.append('地图 (%.2f m/格; 数字=该位姿(朝向 yaw=0)会命中的分支, #=机器人放不下, .=与迷宫内部不连通的空地)' % CELL)
    lines.append('y 从下到上递增, x 从左到右递增; 左下角 = (%.2f, %.2f)' % (x0, y0))
    for row in grid:
        lines.append('  ' + row)
    lines.append('')

    picks = []
    minx, maxx, miny, maxy = min(xs), max(xs), min(ys), max(ys)
    # 候选还要在外墙包围盒内：若迷宫有缺口，连通性判定会把外部空地也算进来
    pool = [c for c in cells if minx <= c[0] <= maxx and miny <= c[1] <= maxy] or cells
    ba = beams(boxes, circles, 0.0, 0.0)
    picks.append(('A基线', (0.0, 0.0, ba, classify(ba['F'], ba['FL'], ba['LF'], ba['FR']))))
    if pool:
        picks.append(('C开阔', max(pool, key=lambda c: min(c[2].values()))))
    d = [c for c in pool if c[2]['F'] > 0.7 and 0.15 <= c[2]['FL'] <= 0.35
         and c[2]['LF'] <= 0.9]
    if not d:   # 若没有能真正走到分支 3 的位置，放宽条件（会被分支 1 抢先）
        d = [c for c in pool if c[2]['F'] > 0.7 and 0.15 <= c[2]['FL'] <= 0.35]
    if d:
        picks.append(('D贴墙', min(d, key=lambda c: abs(c[2]['FL'] - 0.25))))
    b2 = [c for c in pool if c[3] == 2]
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

    print()
    print('地图 %dx%d（%.2f m/格），完整结果已写入 %s' % (nx, ny, CELL, out_path))
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
