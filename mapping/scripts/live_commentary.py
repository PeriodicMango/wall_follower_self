"""Append local file-based observations from a hardware run to its shared terminal log."""

import argparse
import fcntl
import json
from datetime import datetime
from pathlib import Path
import re
import time


COLORS = {"blue": "蓝", "pink": "粉", "green": "绿", "yellow": "黄"}
COLOR_PAIR = r"(?:blue|pink|green|yellow)/(?:blue|pink|green|yellow)"
CAMERA = "/turtlebot_test/camera/compressed"
REASONS = {
    "awaiting new ARM token": "等待启用",
    "ARM removed; new token required": "已解除启用",
    "30 second test complete; new token required": "本次限时测试已结束",
    "guard exited": "保护节点已退出",
    "scan: stale": "激光超时", "odom: stale": "里程计超时", "command: stale": "上游速度指令超时",
    "scan: not received": "等待激光", "odom: not received": "等待里程计", "command: not received": "等待速度指令",
    "scan: timestamp not advancing": "激光时间戳未更新", "odom: timestamp not advancing": "里程计时间戳未更新",
    "scan: all laser ranges invalid": "此帧没有有效的激光测距",
    "scan: unsupported scan geometry or range limits": "激光扫描角度、点数或量程不符",
    "command: non-finite or malformed command": "上游速度指令无效",
    "odom: non-finite odometry": "里程计数据无效",
}


def guard_reasons(reasons):
    return "、".join(REASONS.get(r, r) for r in reasons)


def label(kind):
    return "上" + COLORS[kind.split("/")[0]] + "下" + COLORS[kind.split("/")[1]]


class Reporter:
    def __init__(self, run):
        self.run = run
        self.output = run / "logs" / "实时播报.log"
        self.positions = {}
        self.json_times = {}
        self.seen = set()
        self.guard_state = None
        self.goal = None
        self.known = 0
        self.quiet = False

    def event(self, key, message):
        if key in self.seen:
            return
        self.seen.add(key)
        self.write(message)

    def write(self, message):
        if self.quiet:
            return
        with self.output.open("a", encoding="utf-8") as stream:
            stream.write(f"{datetime.now():%H:%M:%S}  {message}\n")

    def detected(self, kind):
        self.event(("detection", kind), f"相机首次识别双色地标，{label(kind)}。")

    def line(self, line):
        if "Inserted submap (0, 0)" in line:
            self.event("submap", "Cartographer 已开始生成第一张局部地图。")
        if "[ODOM] Robot has left the starting area" in line:
            self.event("leave", "里程计显示机器人已离开巡墙起点。")
        if "[ODOM] Near start!!" in line:
            self.event("return", "巡墙节点检测到机器人已回到起点附近。")
        match = re.search(r"\[MARKER\]\s+(" + COLOR_PAIR + r")\b", line)
        if match:
            self.detected(match[1])
        match = re.search(r"Saved (\d+) markers to (.+)", line)
        if match:
            n = int(match[1])
            self.event(("saved", n), f"记录器已写入 {n}/4 个地标。" + ("地标尚未收齐。" if n < 4 else ""))
        if "Saved the map-frame start pose" in line:
            self.event("startmap", "建图起点的地图坐标已保存。")
        if "Map saved successfully" in line:
            self.event("map_save", "地图保存程序报告写入成功。")
        if "Received a " in line and " map @ " in line and "[amcl]" in line:
            self.event("amclmap", "AMCL 已收到真机导航地图。")
        match = re.search(r"Marker (\d)/4 \((" + COLOR_PAIR + r")\).*?navigating to (\([^)]+\))", line)
        if match:
            self.goal = int(match[1])
            self.event(("goal", self.goal), f"导航第 {self.goal}/4 站，{label(match[2])}，目标坐标 {match[3]} 米。")
        if "Goal succeeded" in line and "[bt_navigator]" in line and self.goal:
            self.arrived(self.goal)
        if "Visited all four markers in file order" in line:
            self.event("complete", "导航任务报告已按文件顺序完成四个地标。")
        if "[ERROR]" in line and "[navigate_markers]" in line:
            reason = line.split("[navigate_markers]:")[-1].strip()
            self.event(("failure", reason), "地标导航任务报告失败，" + reason)

    def arrived(self, number):
        self.event(("arrived", number), f"Nav2 已到达第 {number}/4 站，动作状态 SUCCEEDED。")

    def guard(self, state):
        signature = (state.get("state"), tuple(state.get("reasons", [])))
        if not signature[0] or self.guard_state == signature:
            return
        self.guard_state = signature
        if signature[0] == "armed_ready":
            self.write("速度保护已启用，输入满足放行条件。")
        else:
            reason = guard_reasons(signature[1])
            self.write("速度保护保持零速度。" + reason)

    def observer(self, state):
        latest = state.get("latest", {})
        for kind in state.get("raw_detector_counts", {}):
            if re.fullmatch(COLOR_PAIR, kind):
                self.detected(kind)
        mapped = state.get("mapped_marker_xy", {})
        for kind, xy in mapped.items():
            if re.fullmatch(COLOR_PAIR, kind) and isinstance(xy, list) and len(xy) >= 2:
                self.event(("mapped", kind), f"{label(kind)}已投影到地图，首次观测坐标 ({xy[0]:.2f}, {xy[1]:.2f}) 米。")
        grid = latest.get("/map") or {}
        known = grid.get("known", 0)
        if known and (not self.known or known >= self.known + 2500):
            self.known = known
            self.write(f"已收到 {grid['width']} × {grid['height']} 格地图，已知 {known} 格。")
        payload = latest.get(CAMERA, {}).get("bytes", 0)
        if state.get("camera_topic") == CAMERA and isinstance(payload, int) and payload > 0:
            self.event("camera", f"专用相机话题 {CAMERA} 已收到非空压缩帧，载荷 {payload} 字节。")
        if state.get("four_measured_and_saved"):
            self.event("checked_map_markers", "本次实测的四个地标与保存坐标均已核对。")
        for num, data in enumerate(sorted(state.get("goals", {}).values(), key=lambda g:g["accepted_stamp"]), 1):
            if str(num) in state.get("announced", {}) and data.get("status") == 4:
                self.arrived(num)
        if state.get("four_goals_succeeded"):
            self.event("verified", "观测器已核对四个导航动作全部成功。")

    def tick(self):
        # Launch tees may duplicate per-node logs. Dedup events by meaning and
        # process timestamped messages from both sources in ROS-time order.
        pending = []
        for p in (self.run / "logs").glob("*.log"):
            if p == self.output or p.stat().st_size == 0:
                continue
            identity, offset = self.positions.get(p, (None, 0))
            st = p.stat()
            if st.st_ino != identity or st.st_size < offset:
                offset = 0
            if st.st_size == offset:
                continue
            with p.open("rb") as stream:
                stream.seek(offset)
                for raw in stream:
                    if not raw.endswith(b"\n"):
                        break
                    offset += len(raw)
                    line = raw.decode("utf-8", errors="replace").rstrip()
                    stamp = re.search(r"\[(\d{10}\.\d+)\]", line)
                    pending.append((float(stamp[1]) if stamp else st.st_mtime, line))
            self.positions[p] = (st.st_ino, offset)
        for _, line in sorted(pending, key=lambda pair: pair[0]):
            self.line(line)
        for p in (self.run / "evidence").glob("*.json"):
            st = p.stat()
            if self.json_times.get(p) == st.st_mtime_ns:
                continue
            try:
                data = json.loads(p.read_text())
            except (OSError, ValueError):
                continue
            self.json_times[p] = st.st_mtime_ns
            if p.name == "guard-status.json":
                self.guard(data)
            elif isinstance(data, dict) and "raw_detector_counts" in data:
                self.observer(data)


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument("run", type=Path)
    args.add_argument("--resume", action="store_true", help="Reconstruct already reported state quietly")
    options = args.parse_args()
    reporter = Reporter(options.run.resolve())
    lock = (reporter.output.parent / ".live_commentary.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        raise SystemExit("This run already has a live commentary relay.")
    if options.resume:
        reporter.quiet = True
        reporter.tick()
        reporter.quiet = False
        reporter.seen.discard("camera")
        reporter.json_times.clear()
    else:
        reporter.write("实时播报已接入本次真机日志。")
    while True:
        try:
            reporter.tick()
        except FileNotFoundError:
            pass  # A log was rotated between listing and opening.
        time.sleep(0.8)
