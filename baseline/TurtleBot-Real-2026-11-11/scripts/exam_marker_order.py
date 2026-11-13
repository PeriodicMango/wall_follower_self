"""Read the instructor's visit order; coordinates always come from the selected map."""
import csv
import math
from pathlib import Path


# Type IDs written by the team's wall_follower.landmark.Landmark.toRow().
TYPE_LABELS = ("yellow/pink", "green/pink", "blue/pink",
               "pink/yellow", "pink/green", "pink/blue")
CHINESE = {"黄": "yellow", "绿": "green", "蓝": "blue", "粉": "pink"}


def _measurements(markers):
    known = {}
    for m in markers:
        x, y, kind = m
        if kind not in TYPE_LABELS or kind in known or not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError("实测地标列表包含无效坐标、未知或重复色序。")
        known[kind] = (x, y, kind)
    if not 1 <= len(known) <= 4:
        raise ValueError("本轮需要有 1–4 个不同的实测地标。")
    return known


def _label(value):
    words = [w.strip().lower() for w in value.replace("／", "/").split("/")]
    return "/".join(CHINESE.get(w.removesuffix("色"), w) for w in words)


def read_exam_order(csv_path, markers):
    """Return original measured (x, y, kind) tuples in CSV order, or refuse invalid input.

    The editable order format is a single 'marker' column (upper/lower colour).
    Also accepts a reordered copy of the team's x,y,type_id landmarks.csv.
    In that format each pair of coordinates must still match its original record.
    """
    path = Path(csv_path)
    known = _measurements(markers)
    rows = []
    with path.open(encoding="utf-8-sig", newline="") as stream:
        try:
            reader = csv.reader(stream, strict=True)
            for row in reader:
                if not row or (len(row) == 1 and not row[0].strip()):
                    continue
                rows.append((reader.line_num, [s.strip() for s in row]))
        except csv.Error as exc:
            raise ValueError(f"{path} 第 {reader.line_num} 行 CSV 语法错误：{exc}") from exc

    if not rows:
        raise ValueError(f"{path} 为空，请按老师要求填写地标顺序。")
    header = [field.lower() for field in rows[0][1]]
    cols = len(header)
    if header in (["marker"], ["地标"], ["x", "y", "type"], ["x", "y", "type_id"]):
        rows = rows[1:]
    elif cols not in (1, 3):
        raise ValueError(f"{path} 需要一列 marker，或三列 x,y,type_id。")

    ordered = {}
    for line, row in rows:
        where = f"{path} 第 {line} 行"
        if len(row) != cols:
            raise ValueError(f"{where} 有 {len(row)} 列，期望 {cols} 列。")
        if cols == 1:
            kind = _label(row[0])
        else:
            if row[2] not in ("0", "1", "2", "3", "4", "5"):
                raise ValueError(f"{where} 的 type_id 应为队友定义的 0–5。")
            kind = TYPE_LABELS[int(row[2])]
        if kind not in known:
            raise ValueError(f"{where} 的 {kind!r} 不在本图已记录地标中：{', '.join(known)}。")
        if kind in ordered:
            raise ValueError(f"{where} 重复了 {kind}，每个实测地标只能出现一次。")
        if cols == 3:
            try:
                x, y = map(float, row[:2])
            except ValueError as exc:
                raise ValueError(f"{where} 的坐标不是数字。") from exc
            mx, my, _ = known[kind]
            if not (math.isfinite(x) and math.isfinite(y) and math.hypot(x-mx, y-my) <= 0.001):
                raise ValueError(f"{where} 的坐标和本图 {kind} 不匹配；请移动整行，或改用单列 marker。")
        ordered[kind] = known[kind]
    missing = [k for k in known if k not in ordered]
    if missing:
        raise ValueError(f"{path} 漏掉了本图已记录的地标：{', '.join(missing)}。")
    return list(ordered.values())


def create_order_template(path, markers):
    """Create a new UTF-8 marker-only CSV. Existing teacher edits are never replaced."""
    path = Path(path)
    known = _measurements(markers)
    with path.open("x", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream, lineterminator="\n")
        writer.writerow(["marker"])
        writer.writerows((kind,) for kind in known)
    return path
