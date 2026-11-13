"""A dated, user-operated mapping session on the confirmed physical .103."""
import csv
import fcntl
import json
import math
import os
from pathlib import Path
import select
import signal
import subprocess as sp
import sys
import time
from datetime import datetime

import yaml

from live_commentary import guard_reasons

REAL = Path(__file__).resolve().parents[1]
TOOLS = REAL / "scripts"
PI = TOOLS / "pi_mapping_session.sh"
CAMERA = "/turtlebot_test/camera/compressed"
VARIANT = REAL / "local_variants/wall_follower_2_3"
FOLLOWER = VARIANT / "build/wall_follower_2_3"
FOLLOWER_CONFIG = VARIANT / "wall_follower_2_3.yaml"
COURSE = Path(os.environ.get("TB3_COURSE", Path.home() / "COMP3431/mac-native-gazebo-11.10.2"))
RVIZ = COURSE / "apps/COMP3431 RViz.app/Contents/MacOS/rviz2"
COLORS = {"yellow/pink", "green/pink", "blue/pink", "pink/yellow", "pink/green", "pink/blue"}


def say(line):
    print(f"{datetime.now():%H:%M:%S}  {line}", flush=True)


def guard(action, *, timeout=40):
    sp.run(["/bin/bash", str(PI), action], check=True, timeout=timeout)


def stop(proc, sig=signal.SIGINT, wait=7):
    if proc is None or proc.poll() is not None:
        return
    for signum, timeout in ((sig, wait), (signal.SIGTERM, 3), (signal.SIGKILL, 2)):
        try:
            os.killpg(proc.pid, signum)
            proc.wait(timeout)
            return
        except ProcessLookupError:
            return
        except sp.TimeoutExpired:
            continue


def artifacts(maps, prefix, markers):
    meta = yaml.safe_load(prefix.with_suffix(".yaml").read_text())
    raster = maps / meta["image"]
    if meta["resolution"] <= 0 or not raster.exists() or raster.stat().st_size < 16:
        raise ValueError("地图 YAML 或图像不完整")
    with markers.open(newline="") as f:
        rows = [row for row in csv.reader(f) if row]
    if not rows or len(rows[0]) != 3 or not all(math.isfinite(float(v)) for v in rows[0]):
        raise ValueError("Marker 文件缺少里程计起点")
    colors = []
    for row in rows[1:]:
        if (len(row) != 3 or row[2] not in COLORS or row[2] in colors or
                not all(math.isfinite(float(v)) for v in row[:2])):
            raise ValueError(f"无效的 Marker 记录：{row}")
        colors.append(row[2])
    return raster, colors


def main():
    if not sys.stdin.isatty():
        sys.exit("请直接在 Mac 的交互式终端运行此命令。")
    if os.environ.get("ROS_DOMAIN_ID") != "3" or os.environ.get("ROS_LOCALHOST_ONLY") != "0":
        sys.exit("真机环境未加载，请通过 真机巡墙建图.command 运行。")
    if not os.access(FOLLOWER, os.X_OK):
        sys.exit(f"本机视觉循墙程序尚未编译：{FOLLOWER}")
    try:
        param_text = FOLLOWER_CONFIG.read_text()
        params = yaml.safe_load(param_text)["wall_follower_node"]["ros__parameters"]
        for key in ("front_block_dist", "left_open_dist", "linear_speed_m_s"):
            if type(params[key]) is not float or not math.isfinite(params[key]) or params[key] <= 0:
                raise ValueError(f"{key} 应为正的小数，例如 0.70")
    except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
        sys.exit(f"请先检查 {FOLLOWER_CONFIG}：{exc}")
    runs = REAL / "runs"
    runs.mkdir(exist_ok=True)
    lock = (runs / ".manual_mapping.lock").open("a")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("另一个手动建图终端仍在运行；请回到原终端。")
    stamp = datetime.now().strftime("%Y-%m-%d_%H%M%S")
    run = runs / ("Real_103_" + stamp)
    maps, logs = run / "maps", run / "logs"
    maps.mkdir(parents=True)
    logs.mkdir()
    used_params = run / "wall_follower_2_3.used.yaml"
    used_params.write_text(param_text)
    os.environ["ROS_LOG_DIR"] = str(logs)
    os.chdir(run)
    markers = maps / f"markers_Real_103_{stamp}.txt"
    prefix = maps / f"maze_Real_103_{stamp}"
    config = REAL / "config/真机巡墙彩色Marker.rviz"
    procs = {}
    handles = []
    requests = []
    lease = None

    def start(name, *args, **kw):
        f = (logs / (name + ".log")).open("ab", buffering=0)
        handles.append(f)
        procs[name] = sp.Popen([str(x) for x in args], stdout=f, stderr=sp.STDOUT,
                               start_new_session=True, **kw)
        return procs[name]

    def interrupt(signum, frame):
        requests.append(signum)

    for s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(s, interrupt)
    prepared = False
    may_move = False
    result = 1
    say(f"本次真机目录：{run}")
    say("本机版本：wall_follower 2.3 用雷达循墙；See Marker 独立记录地标，固定后拉 = 0.00 米。")
    say(f"前方受阻阈值 {params['front_block_dist']:g} m，配置副本 {used_params.name}")
    try:
        say("正在连接已配对的 TurtleBot，准备传感器；车轮保持停止。")
        guard("prepare", timeout=90)
        prepared = True
        if requests:
            return 1
        # The camera preview can stay open during sensor checks.
        preview = TOOLS / "vision/open_preview.sh"
        if preview.exists():
            with (logs / "preview-open.log").open("w") as pv:
                sp.Popen(["/bin/bash", str(preview)], stdout=pv, stderr=sp.STDOUT,
                         start_new_session=True, stdin=sp.DEVNULL)
        start("cartographer", "ros2", "launch", "turtlebot3_cartographer", "cartographer.launch.py",
              "use_sim_time:=false", "use_rviz:=false")
        start("detector", sys.executable, "-u", TOOLS / "vision/hardware_see_marker.py",
              "--topic", CAMERA, "--evidence-dir", run / "vision")
        rec = start("recorder", sys.executable, "-u", TOOLS / "vision/hardware_marker_recorder.py",
                    "--marker-file", markers)
        start("rviz", RVIZ, "-d", config, "--ros-args", "-r", "__node:=robot103_manual_mapping_rviz",
              "-p", "use_sim_time:=false")
        # The pipe closes if this parent is killed. The supervisor then stops the
        # command publisher, so the Pi's stale-command guard holds zero.
        lease_r, lease = os.pipe()
        try:
            wall = start("follower", sys.executable, TOOLS / "follower_lease.py", str(lease_r), FOLLOWER,
                         "--ros-args", "--params-file", used_params, "-p", "use_sim_time:=false", "-r",
                         "cmd_vel:=/turtlebot_test/wall_cmd_vel", pass_fds=(lease_r,))
        finally:
            os.close(lease_r)
        check = start("preflight", sys.executable, TOOLS / "check_mapping_inputs.py", "--timeout", "25",
                      "--follower-binary", FOLLOWER,
                      "--output", run / "mapping_preflight.json")
        while check.poll() is None and not requests:
            time.sleep(.1)
        if requests:
            return 1
        if check.returncode != 0:
            print((logs / "preflight.log").read_text(errors="replace")[-3500:])
            raise RuntimeError("未放行；请查看上面的传感器或同名节点提示")
        for name in ("cartographer", "recorder", "detector", "follower", "rviz"):
            if procs[name].poll() is not None:
                raise RuntimeError(f"{name} 启动失败：{logs / (name + '.log')}")
        say("真机地图与压缩图像已就绪，正在启用循墙。")
        may_move = True
        try:
            guard("arm")
        except sp.CalledProcessError as exc:
            try:
                info = json.loads(sp.check_output(["/bin/bash", str(PI), "status"], text=True, timeout=10))
                if info.get("fresh") and info.get("guard", {}).get("reasons"):
                    say("速度保护报告" + guard_reasons(info["guard"]["reasons"]) + "。")
            except (OSError, ValueError, sp.SubprocessError):
                pass  # The original arm command already printed its error.
            raise RuntimeError("树莓派未确认启用巡墙，请查看终端中的错误信息。") from exc
        say("循墙已启用。按回车停车并保存；检测到新色序时，下面会显示坐标。")
        if requests:
            say("收到停止指令。")
        with (logs / "recorder.log").open(errors="replace") as log:
            while not requests:
                for line in log.readlines():
                    if "[MAPPED]" in line:
                        say(line.partition("[MAPPED]")[2].strip())
                if select.select([sys.stdin], [], [], .2)[0]:
                    sys.stdin.readline()
                    break
                down = next((n for n in ("cartographer", "detector", "recorder", "follower")
                             if procs[n].poll() is not None), None)
                if down:
                    say(f"{down} 已结束，停车并保存本次已获取的数据。")
                    break
        os.close(lease)
        lease = None
        stop(wall, wait=2)
        try:
            guard("disarm", timeout=14)
            may_move = False
        except (sp.TimeoutExpired, sp.CalledProcessError):
            say("已停止巡墙指令；未取得树莓派停车回执，请现场确认。")
        stop(rec, wait=8)  # recorder flushes actual observations while SLAM TF still exists
        say("正在保存占用地图与 Marker 坐标……")
        cmd = ["ros2", "run", "nav2_map_server", "map_saver_cli", "-f", str(prefix), "--free", "0.15",
               "--ros-args", "-p", "save_map_timeout:=12.0", "-p", "use_sim_time:=false"]
        with (logs / "save-map.log").open("w") as f:
            sp.run(cmd, stdout=f, stderr=sp.STDOUT, timeout=22, check=True)
        raster, colors = artifacts(maps, prefix, markers)
        say(f"地图：{prefix.with_suffix('.yaml')} 及 {raster.name}")
        say(f"Marker：{markers}，实测 {len(colors)} 个（{', '.join(colors) or '无'}）。")
        if not markers.with_suffix(".start_map.txt").exists():
            say("未取得地图坐标系的起点；不要直接用此文件自动初始化导航。")
        if len(colors) != 4:
            say("本次不是完整的四 Marker 地图；已保留部分结果。")
        elif not may_move and markers.with_suffix(".start_map.txt").exists():
            # SLAM is still publishing, and the guard has confirmed stopped.
            dest = maps / "navigation_start"
            try:
                status = json.loads(sp.check_output(["/bin/bash", str(PI), "status"], text=True, timeout=12))
                boot = status.get("boot_id") or status.get("pi_boot_id")
                if not boot:
                    raise ValueError("缺少树莓派启动标识")
                with (logs / "handoff.log").open("w") as hf:
                    sp.run([sys.executable, str(TOOLS / "observe_hardware.py"), "--duration", "6",
                            "--markers", str(markers), "--handoff", str(dest), "--camera", CAMERA,
                            "--output", str(run / "handoff_status.json")], stdout=hf, stderr=sp.STDOUT,
                           timeout=15, check=True)
                ready = dict(map=str(prefix.with_suffix(".yaml").relative_to(run)), markers=str(markers.relative_to(run)),
                             handoff=str((dest / markers.name).relative_to(run)), saved_local=stamp, pi_boot_id=boot)
                (run / "ready_for_navigation.json").write_text(json.dumps(ready, indent=2)+"\n")
                say("已保存四个 Marker 和停车时的地图位姿。直接导航前请保持机器人原位。")
            except (OSError, ValueError, sp.SubprocessError) as exc:
                say(f"地图已保存，未生成快速导航起点：{exc}。详见 {logs / 'handoff.log'}")
        if colors and not requests and procs["rviz"].poll() is None:
            start("saved_markers", sys.executable, "-u", TOOLS / "vision/show_saved_hardware_markers.py",
                  "--marker-file", markers, "--ros-args", "-p", "use_sim_time:=false")
        result = 0
        if not requests and procs["rviz"].poll() is None:
            say("车已停止。可在 RViz 查看本次结果；再按回车关闭本次建图窗口。")
            while not requests and procs["rviz"].poll() is None:
                if select.select([sys.stdin], [], [], .3)[0]:
                    sys.stdin.readline()
                    break
    except (OSError, ValueError, RuntimeError, sp.SubprocessError, KeyError) as exc:
        say(f"未完成：{exc}。已有文件和详细日志保留在 {run}")
    finally:
        if lease is not None:
            os.close(lease)
        stop(procs.get("follower"), wait=2)
        if prepared and may_move:
            try:
                guard("disarm", timeout=14)
            except (OSError, sp.SubprocessError):
                say("未收到真机停车回执；巡墙指令已停止。")
        for name in ("preflight", "recorder", "saved_markers", "detector", "cartographer", "rviz"):
            stop(procs.get(name))
        for h in handles:
            h.close()
        lock.close()
    return result


if __name__ == "__main__":
    sys.exit(main())
