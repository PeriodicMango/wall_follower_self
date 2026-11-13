"""View a saved .103 OccupancyGrid and its recorded markers in RViz."""
import argparse
from datetime import datetime
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

from PIL import Image
import yaml

from manual_mapping import REAL, RVIZ, TOOLS, say
from real_run_catalog import run_file, select_run, validate_grid


def view_config(map_yaml, output):
    source = yaml.safe_load((REAL / "config/真机导航彩色Marker.rviz").read_text())
    manager = source["Visualization Manager"]
    manager["Displays"] = [d for d in manager["Displays"] if d["Name"] in ("Physical maze", "Colored landmarks")]
    manager["Tools"] = [t for t in manager["Tools"] if t["Class"] != "rviz_default_plugins/SetInitialPose"]
    meta = yaml.safe_load(map_yaml.read_text())
    with Image.open(map_yaml.parent / meta["image"]) as raster:
        w, h = [n * meta["resolution"] for n in raster.size]
    x, y, yaw = meta["origin"]
    x += (w * math.cos(yaw) - h * math.sin(yaw)) / 2
    y += (w * math.sin(yaw) + h * math.cos(yaw)) / 2
    # The landmarks have vertically stacked colour bands, best seen at a slant.
    view = manager["Views"]["Current"]
    view.update({"Pitch": 0.95, "Distance": max(w, h) * 1.7,
                 "Focal Point": {"X": x, "Y": y, "Z": 0.05}})
    output.write_text(yaml.safe_dump(source, sort_keys=False, allow_unicode=True))


def viewer(map_yaml, markers, log, parent):
    import rclpy
    from ament_index_python.packages import get_package_prefix
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from rclpy.signals import SignalHandlerOptions

    children = {}

    def start(name, *args):
        with (log / (name + ".log")).open("wb") as stream:
            children[name] = sp.Popen([str(a) for a in args], stdout=stream, stderr=sp.STDOUT,
                                      stdin=sp.DEVNULL)
        return children[name]

    def check():
        if os.getppid() != parent:
            raise InterruptedError
        for name in ("map_server", "rviz", "landmarks"):
            child = children.get(name)
            if child is not None and child.poll() is not None:
                if name == "rviz" and child.returncode == 0:
                    raise InterruptedError
                raise RuntimeError(f"{name} 已退出，请查看 {log / (name + '.log')}")

    # This service is private to this viewer, even when another machine uses /map_server.
    service = f"/robot103_view_{os.getpid()}/change_state"
    node = None
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    try:
        node = rclpy.create_node("robot103_saved_map_lifecycle", enable_rosout=False, start_parameter_services=False)
        client = node.create_client(ChangeState, service)
        until = time.monotonic() + 1.5
        while time.monotonic() < until:
            check()
            rclpy.spin_once(node, timeout_sec=.12)
        if node.get_publishers_info_by_topic("/map") or ("map_server", "/") in node.get_node_names_and_namespaces():
            raise RuntimeError("已有地图发布者或 /map_server 正在运行，请先关闭该地图任务。")
        binary = Path(get_package_prefix("nav2_map_server")) / "lib/nav2_map_server/map_server"
        start("map_server", binary, "--ros-args", "-p", "yaml_filename:=" + str(map_yaml),
              "-p", "use_sim_time:=false", "-r", "/map_server/change_state:=" + service)
        for transition in (Transition.TRANSITION_CONFIGURE, Transition.TRANSITION_ACTIVATE):
            deadline = time.monotonic() + 30
            while not client.wait_for_service(timeout_sec=.2):
                check()
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"等待本次地图服务器生命周期服务超时，详见 {log / 'map_server.log'}")
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            while not future.done():
                check()
                rclpy.spin_once(node, timeout_sec=.15)
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"地图生命周期响应超时，详见 {log / 'map_server.log'}")
            if not future.result().success:
                raise RuntimeError(f"地图服务器拒绝切换状态，详见 {log / 'map_server.log'}")
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()

    rviz_config = log / (map_yaml.stem + ".rviz")
    view_config(map_yaml, rviz_config)
    start("rviz", RVIZ, "-d", rviz_config, "--ros-args", "-r", "__node:=robot103_saved_map_rviz",
          "-p", "use_sim_time:=false")
    if markers.is_file() and len(markers.read_text().strip().splitlines()) > 1:
        start("landmarks", sys.executable, "-Bu", TOOLS / "vision/show_saved_hardware_markers.py", "--marker-file", markers)
    verify = start("verify", sys.executable, "-Bu", TOOLS / "verify_loaded_real_map.py", "--map", map_yaml, "--timeout", "30")
    while verify.poll() is None:
        check()
        time.sleep(.2)
    details = (log / "verify.log").read_text(errors="replace")
    print(details, end="", flush=True)
    if verify.returncode:
        raise RuntimeError(f"地图没有通过核验，详见 {log / 'verify.log'}")
    report = json.loads(details.splitlines()[-1])
    (log / "verified-map.json").write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    actual = report["actual"]
    say(f"已核实 /map 来自 {map_yaml.name}，共 {actual['width']} × {actual['height']} 格，逐格吻合。")
    rviz_event = f"Trying to create a map of size {actual['width']} x {actual['height']} using "
    with (log / "rviz.log").open(errors="replace") as rviz_log:
        while True:
            check()
            if rviz_event in rviz_log.read():
                say(f"RViz 已收到 {actual['width']} × {actual['height']} 的地图。按回车关闭。")
                break
            time.sleep(.3)
    while True:
        check()
        time.sleep(.3)


def stop_group(pid):
    for signum, delay in ((signal.SIGINT, 2), (signal.SIGTERM, 1), (signal.SIGKILL, 0)):
        try:
            os.killpg(pid, signum)
            end = time.monotonic() + delay
            while time.monotonic() < end:
                os.killpg(pid, 0)
                time.sleep(.07)
        except ProcessLookupError:
            return


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", nargs="?", help="Real run directory or dated map YAML")
    opts = parser.parse_args()
    if not sys.stdin.isatty() or os.environ.get("ROS_DOMAIN_ID") != "3":
        sys.exit("请在交互式 Mac 终端通过 真机查看已存地图.command 运行。")
    runs = REAL / "runs"
    with (runs / ".manual_mapping.lock").open("a") as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            sys.exit("另一个真机建图、导航或看图终端仍在运行。请先在该终端按回车结束。")
        try:
            run, selected = select_run(runs, opts.run)
            map_yaml = run_file(run, Path("maps") / f"maze_{run.name}.yaml")
            if selected is not None and selected not in (run, map_yaml):
                raise ValueError(f"请选择真机记录目录或 {map_yaml}")
            validate_grid(run, map_yaml)
        except (OSError, ValueError) as exc:
            sys.exit(str(exc))
        markers = map_yaml.parent / f"markers_{run.name}.txt"
        log = run / datetime.now().strftime("看图_Real_%Y-%m-%d_%H%M%S_%f")
        log.mkdir()
        os.environ["ROS_LOG_DIR"] = str(log)
        say(f"加载 {map_yaml}。日志 {log}。按回车可随时关闭。")
        parent = os.getppid()
        stopped = []
        for sig in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: stopped.append(True))
        # One group for the worker and its exact children. If this terminal is
        # killed abruptly, the worker detects reparenting and closes its group.
        worker = sp.Popen([sys.executable, "-Bu", __file__, "--worker", str(map_yaml), str(markers),
                           str(log), str(os.getpid())], start_new_session=True, stdin=sp.DEVNULL)
        try:
            while worker.poll() is None:
                if stopped or os.getppid() != parent or select.select([sys.stdin], [], [], .2)[0]:
                    return 0
            return worker.returncode
        finally:
            stop_group(worker.pid)
            worker.wait()


if __name__ == "__main__":
    if sys.argv[1:2] == ["--worker"]:
        try:
            viewer(*map(Path, sys.argv[2:5]), int(sys.argv[5]))
        except (KeyboardInterrupt, InterruptedError):
            pass
        except (OSError, RuntimeError, ValueError) as exc:
            say(str(exc))
            sys.exit(1)
        finally:
            if os.getppid() != int(sys.argv[5]):
                signal.signal(signal.SIGTERM, signal.SIG_IGN)
                os.killpg(os.getpgrp(), signal.SIGTERM)
    else:
        sys.exit(main())
