"""Locate the physical .103 map, its recorded landmarks and measured navigation start."""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import re

from PIL import Image
import yaml


RUN_NAME = re.compile(r"Real_103_(\d{4}-\d{2}-\d{2}_\d{6})")
COLORS = {"yellow/pink", "green/pink", "blue/pink", "pink/yellow", "pink/green", "pink/blue"}
READY = "ready_for_navigation.json"


def finite(values, count):
    if not isinstance(values, (list, tuple)):
        return False
    try:
        return len(values) == count and all(math.isfinite(float(n)) for n in values)
    except (TypeError, ValueError, OverflowError):
        return False


def run_file(run, relative):
    path = run / relative
    if run not in path.resolve().parents or not path.is_file():
        raise ValueError(f"此真机记录中找不到文件 {path}")
    return path


def csv_rows(path):
    with path.open(newline="") as stream:
        return [r for r in csv.reader(stream) if r]


def measured_rows(path, allow_partial=False):
    rows = csv_rows(path)
    valid_count = 2 <= len(rows) <= 5 if allow_partial else len(rows) == 5
    if not valid_count or not finite(rows[0], 3):
        required = "一到四行" if allow_partial else "四行"
        raise ValueError(f"{path} 需要一行有效起点和{required}实测地标")
    labels = set()
    for row in rows[1:]:
        if len(row) != 3 or not finite(row[:2], 2) or row[2] not in COLORS or row[2] in labels:
            raise ValueError(f"{path} 中有无效坐标、色序或重复地标")
        labels.add(row[2])
    return rows


def marker_rows(run, path):
    rows = measured_rows(path)
    sidecar = run_file(run, path.with_suffix(".start_map.txt").relative_to(run))
    pose = csv_rows(sidecar)
    if len(pose) != 1 or not finite(pose[0], 3):
        raise ValueError(f"{sidecar} 缺少有效的地图起点 x、y、yaw")
    return rows, pose[0]


def select_run(runs_dir, requested=None):
    selected = None
    root = Path(runs_dir).expanduser().resolve()
    if requested is None:
        candidates = sorted(p for p in root.glob("Real_103_*") if p.is_dir() and RUN_NAME.fullmatch(p.name))
        if not candidates:
            raise ValueError("还没有带日期的 .103 真机建图记录，请先完成真机建图")
        run = candidates[-1]
    else:
        selected = Path(requested).expanduser()
        if not selected.is_absolute():
            selected = selected if selected.exists() else root / selected
        selected = selected.absolute()
        if selected == root or root not in selected.parents:
            raise ValueError(f"请指定 {root} 下的真机记录")
        run = root / selected.relative_to(root).parts[0]
    match = RUN_NAME.fullmatch(run.name)
    if not match or run.parent != root or run.resolve() != run or not run.is_dir():
        raise ValueError(f"不符合带日期的 .103 真机目录格式：{run}")
    datetime.strptime(match[1], "%Y-%m-%d_%H%M%S")
    return run, selected


def load_run(runs_dir, requested=None):
    """Return (run, map_yaml, original_markers, handoff_markers), or raise ValueError."""
    run, selected = select_run(runs_dir, requested)
    if not (run / READY).is_file():
        grid_file = run / "maps" / f"maze_{run.name}.yaml"
        record = run / "maps" / f"markers_{run.name}.txt"
        if not (grid_file.is_file() and grid_file.with_suffix(".pgm").is_file()):
            raise ValueError(f"{run.name} 尚未保存完整的地图 YAML 和 PGM。")
        if not record.is_file():
            raise ValueError(f"{run.name} 已保存地图，但尚无地标坐标文件。")
        labels = [r[2] for r in csv_rows(record)[1:] if len(r)==3 and r[2] in COLORS and finite(r[:2], 2)]
        unique = list(dict.fromkeys(labels))
        if len(unique) != 4:
            raise ValueError(f"{run.name} 已保存地图，实测地标 {len(unique)}/4（{', '.join(unique) or '无'}）。"
                             "四标自动导航需要四种不同色序的地图坐标。")
        raise ValueError(f"{run.name} 已保存地图和四个地标，但缺少可用于自动定位的停车位姿或交接记录。")
    meta = json.loads(run_file(run, READY).read_text())
    if not isinstance(meta, dict) or not isinstance(meta.get("saved_local"), str) or not meta["saved_local"].strip():
        raise ValueError(f"{run.name} 的完成记录缺少保存时间")
    expected = {
        "map": Path("maps") / f"maze_{run.name}.yaml",
        "markers": Path("maps") / f"markers_{run.name}.txt",
        "handoff": Path("maps/navigation_start") / f"markers_{run.name}.txt",
    }
    for key, path in expected.items():
        if meta.get(key) != str(path):
            raise ValueError(f"{run.name} 的 {key} 未指向本次真机建图文件 {path}")
    map_yaml, original, handoff = [run_file(run, p) for p in expected.values()]
    if requested is not None and selected not in (run, run / READY, map_yaml, original, handoff):
        raise ValueError(f"指定路径不是该真机记录、完成记录、地图或地标文件：{selected}")
    validate_grid(run, map_yaml)
    original_rows, _ = marker_rows(run, original)
    handoff_rows, map_pose = marker_rows(run, handoff)
    if original_rows[1:] != handoff_rows[1:]:
        raise ValueError("停车交接文件中的四行地标与建图原件不完全一致")
    measurement = json.loads(run_file(run, handoff.parent.relative_to(run) / "measured_pose.json").read_text())
    if not isinstance(measurement, dict):
        raise ValueError("停车位姿证据格式无效")
    for key, values in (("odom_pose", handoff_rows[0]), ("map_pose", map_pose)):
        if not finite(measurement.get(key), 3) or list(map(float, measurement[key])) != list(map(float, values)):
            raise ValueError(f"交接的 {key} 与停车时的测量记录不一致")
    return run, map_yaml, original, handoff


def validate_grid(run, map_yaml):
    try:
        grid = yaml.safe_load(map_yaml.read_text())
    except yaml.YAMLError as exc:
        raise ValueError(f"{map_yaml} 格式无效") from exc
    if not isinstance(grid, dict) or not finite([grid.get("resolution")], 1) or float(grid["resolution"]) <= 0:
        raise ValueError(f"{map_yaml} 缺少有效地图分辨率")
    if not finite(grid.get("origin"), 3) or not isinstance(grid.get("image"), str):
        raise ValueError(f"{map_yaml} 缺少有效地图原点或栅格图像")
    raster = run_file(run, map_yaml.with_suffix(".pgm").relative_to(run))
    if (map_yaml.parent / grid["image"]).resolve() != raster.resolve():
        raise ValueError(f"{map_yaml} 的栅格图像不属于本次地图")
    try:
        with Image.open(raster) as image:
            image.load()
            grayscale_pgm = image.format == "PPM" and image.mode == "L"
    except (OSError, ValueError) as exc:
        raise ValueError(f"{raster} 图像损坏或未写完") from exc
    if not grayscale_pgm:
        raise ValueError(f"{raster} 不是建图程序生成的灰度 PGM")


def load_measured_run(runs_dir, requested=None):
    """Return the selected physical map and 1–4 measured marker labels in file order."""
    run, selected = select_run(runs_dir, requested)
    map_yaml = run_file(run, Path("maps") / f"maze_{run.name}.yaml")
    original = run_file(run, Path("maps") / f"markers_{run.name}.txt")
    if selected is not None and selected not in (run, map_yaml, original):
        raise ValueError(f"请选择本次真机记录目录、地图 YAML 或实测地标文件：{run}")
    validate_grid(run, map_yaml)
    rows = measured_rows(original, allow_partial=True)
    return run, map_yaml, original, [row[2] for row in rows[1:]]
