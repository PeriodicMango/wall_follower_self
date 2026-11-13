"""Prepare a teacher mission while the physical guard still holds zero."""
import json
import os
from pathlib import Path
import sys
import time

from manual_mapping import PI, TOOLS, say, stop
from placement_gate import PlacementChanged, current, permit, publish, read


def prepare(log, measured, start, tick, procs, ready_args, marker_args, begin_watchdog):
    status = log / "auto_start_status.json"
    countdown = log / "countdown_request.json"
    arm_request = log / "arm_request.json"

    def step(g=None):
        tick()
        if procs["localization"].poll() is not None:
            raise RuntimeError("起点识别程序已退出：" + (log/"localization.log").read_text(errors="replace")[-600:])
        if procs["nav2"].poll() is not None:
            raise RuntimeError("Nav2 已退出，请看 nav2.log")
        state = read(status)
        if g is not None and not current(state, g):
            raise PlacementChanged("出发前的位置已变化或传感器数据已过期")
        return state

    def log_size(name):
        path = log / name
        return path.stat().st_size if path.exists() else 0

    while True:
        state = step()
        pose = read(measured)
        g = pose.get("generation")
        if not current(state, g):
            continue
        anchor = log / f"start_pose_{g}.json"
        publish(anchor, pose)
        try:
            args = list(ready_args)
            args[args.index("--handoff") + 1] = str(anchor)
            check = start("preflight", *args)
            while check.poll() is None:
                step(g)
            step(g)
            if check.returncode:
                raise RuntimeError((log/"preflight.log").read_text(errors="replace")[-1400:])
            offset = log_size("mission.log")
            mission = start("mission", sys.executable, "-Bu", TOOLS/"navigate_saved_markers.py", *marker_args,
                            "--pose-file", anchor, "--placement-status", status, "--countdown-request", countdown,
                            "--placement-generation", g)
            say("地图及定位已核对。正在规划第一站；首段可达后，终端会开始三秒倒计时。")
            with (log / "mission.log").open("rb") as events:
                events.seek(offset)
                deadline = time.monotonic() + 85
                while True:
                    state = step(g)
                    if mission.poll() is not None:
                        if mission.returncode == 75:
                            raise PlacementChanged("首段规划时起点定位已改变")
                        raise RuntimeError("首段任务已结束：" + (log/"mission.log").read_text(errors="replace")[-800:])
                    if any(b"navigating to (" in line for line in events.readlines()):
                        break
                    if time.monotonic() > deadline:
                        raise TimeoutError("首段路线或倒计时准备超时")
            if not permit(state, g):
                raise PlacementChanged("首个目标已接受，但起点确认已失效")
            # SSH validates health and source topology with wheels held. Only a new
            # matched scan *after* the Pi's unique challenge may release this goal.
            arm_offset = log_size("arming.log")
            arm = start("arming", "/bin/bash", PI, "arm-nav-confirmed", interactive=True)
            with (log / "arming.log").open("rb") as replies:
                replies.seek(arm_offset)
                deadline = time.monotonic() + 20
                challenge = None
                while not challenge:
                    step(g)
                    if arm.poll() is not None:
                        raise RuntimeError("树莓派未放行：" + (log/"arming.log").read_text(errors="replace")[-800:])
                    if mission.poll() is not None:
                        raise RuntimeError("车轮放行前首段任务已结束")
                    for line in replies.readlines():
                        try:
                            challenge = json.loads(line).get("placement_challenge")
                        except (UnicodeError, ValueError, AttributeError):
                            continue
                        if challenge:
                            break
                    if time.monotonic() > deadline:
                        raise TimeoutError("等待树莓派确认指令来源超时")
            publish(arm_request, {"generation": g, "token": challenge})
            deadline = time.monotonic() + 2.
            while True:
                state = step(g)
                if mission.poll() is not None:
                    raise RuntimeError("车轮放行前首段任务已结束")
                if arm.poll() is not None or time.monotonic() >= deadline:
                    raise RuntimeError("未在放行窗口内取得新的起点激光确认，车轮未启用。")
                if permit(state, g, challenge):
                    until = min(state["updated_unix"] + .9, state["scan_stamp"] + 1.0)
                    if until - time.time() > .1:
                        break
            begin_watchdog(mission)
            arm.stdin.write((json.dumps({"challenge": challenge, "valid_until": until}) + "\n").encode())
            arm.stdin.close()
            return mission, check, offset, g
        except PlacementChanged as exc:
            # No commit has been sent; discard the planned goal and start over from
            # a new placement. The Pi sees EOF, never a reusable authorization.
            arm = procs.get("arming")
            if arm and arm.stdin and not arm.stdin.closed:
                arm.stdin.close()
            for label in ("mission", "preflight", "arming"):
                proc = procs.pop(label, None)
                if proc:
                    stop(proc, wait=2)
            say(f"{exc}。车轮保持未启用；等待重新放回起点。")
