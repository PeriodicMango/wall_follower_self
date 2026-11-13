"""User-operated four-marker navigation from the last physical mapping stop."""
import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import select
import signal
import subprocess as sp
import sys
import time

from auto_exam_departure import prepare as prepare_exam_departure
from manual_mapping import PI, REAL, RVIZ, TOOLS, say, stop
from placement_gate import read as read_placement, current as current_placement
from real_run_catalog import load_run, load_measured_run, run_file

def route_text(colors):
    cn = {"yellow": "黄", "pink": "粉", "blue": "蓝", "green": "绿"}
    return " → ".join(["起点", *["/".join(cn[x] for x in color.split("/")) for color in colors], "回到起点"])


def read_pi_status(tick):
    """Retry a short network outage only during the read-only status check."""
    target = "192.168.1.103"
    connect_error = f"ssh: connect to host {target} port 22: "
    retryable = {connect_error + error for error in (
        "Operation timed out", "Connection timed out", "No route to host",
        "Network is unreachable", "Network is down", "Connection refused",
    )}
    attempts = 6
    for attempt in range(1, attempts + 1):
        # Inherit stdin and the controlling terminal so OpenSSH can prompt for a password.
        result = sp.run(["bash", str(PI), "status"], capture_output=True, text=True, timeout=15)
        tick()
        if result.returncode == 0:
            return json.loads(result.stdout)
        reason = result.stderr.strip() or f"退出码 {result.returncode}"
        if result.returncode != 255 or not retryable.intersection(reason.splitlines()):
            raise RuntimeError(f"读取树莓派状态失败。{reason}")
        if attempt == attempts:
            raise RuntimeError(f"尝试 {attempts} 次仍无法通过 SSH 连接 {target}。请确认树莓派已连上同一局域网。\n{reason}")
        say(f"暂时无法通过 SSH 连接 {target}（第 {attempt}/{attempts} 次）。2 秒后重试，按回车可取消。")
        retry_at = time.monotonic() + 2
        while time.monotonic() < retry_at:
            tick()



def main():
    arg = argparse.ArgumentParser(description=__doc__)
    arg.add_argument("run", nargs="?", type=Path, help="optional explicit Real run directory")
    arg.add_argument("--recorded", action="store_true", help="visit the markers actually measured on this map, after RViz localization")
    arg.add_argument("--shuffle", action="store_true", help="randomize the recorded marker order")
    arg.add_argument("--seed", type=int, help="reproduce a shuffled order")
    arg.add_argument("--exam", action="store_true", help="按指定 CSV 顺序访问，再回原起点")
    arg.add_argument("--order-csv", type=Path, help="考核顺序 CSV，默认读取该轮的 maps/landmarks.csv")
    arg.add_argument("--preview", action="store_true", help="只校验考核顺序和起点，不访问机器人")
    arg.add_argument("--expect-count", type=int, choices=(1,2,3,4), help="考核规定的地标数，不符即停止")
    arg.add_argument("--rviz-start", action="store_true", help="手动定位后点击 2D Goal Pose 启动整条考核路线")
    opts = arg.parse_args()
    if opts.rviz_start and not opts.exam:
        arg.error("--rviz-start 仅用于考核导航")
    auto_exam = opts.exam and not opts.rviz_start
    if opts.exam and (not opts.recorded or opts.shuffle or opts.seed is not None):
        arg.error("考核导航须使用 CSV 的顺序，不接受随机模式或随机种子")
    if (opts.order_csv or opts.preview or opts.expect_count is not None) and not opts.exam:
        arg.error("--order-csv、--preview、--expect-count 仅用于考核模式")
    if opts.order_csv:
        opts.order_csv = opts.order_csv.expanduser().resolve()
    if (opts.shuffle and not opts.recorded) or (opts.seed is not None and not opts.shuffle):
        arg.error("--shuffle 需要 --recorded，--seed 需要 --shuffle")
    if (not sys.stdin.isatty() and not opts.preview) or os.environ.get("ROS_DOMAIN_ID") != "3":
        sys.exit("请在 Mac 交互式终端通过 真机四标导航.command 运行。")
    runs = REAL / "runs"
    lock = (runs / ".manual_mapping.lock").open("a")
    try:
        if not opts.preview:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("另一个建图、看图或导航终端还在运行，请先在原终端按回车结束。")
    try:
        if opts.recorded:
            run, map_yaml, original, colors = load_measured_run(runs, opts.run)
            total = len(colors)
            if opts.exam:
                if opts.expect_count is not None and total != opts.expect_count:
                    raise ValueError(f"本次要求 {opts.expect_count} 个地标，这张地图记录了 {total} 个。")
                order_source = opts.order_csv or run_file(run, original.parent.relative_to(run) / "landmarks.csv")
                home_file = run_file(run, original.with_suffix(".start_map.txt").relative_to(run))
        else:
            run, map_yaml, original, handoff = load_run(runs, opts.run)
            total = 4
    except (OSError, ValueError, RuntimeError) as exc:
        sys.exit(f"未启动导航：{exc}")
    if opts.preview:
        args = [sys.executable, "-Bu", TOOLS / "navigate_saved_markers.py", "--marker-file", original,
                "--order-csv", order_source, "--home-file", home_file, "--return-to-start", "--list-only"]
        result = sp.run(args, capture_output=True, text=True)
        if result.returncode:
            sys.exit(result.stderr.strip() or "考核 CSV 或起点无效")
        order = json.loads(result.stdout)
        say(f"{run.name}；{total} 个实测地标；顺序文件：{order_source}")
        say(route_text([r['kind'] for r in order['order']]))
        say(f"返回建图起点（map 坐标系）：{home_file.name}。本次仅预览，未连接机器人。")
        return 0
    meta = {} if opts.recorded else json.loads((run / "ready_for_navigation.json").read_text())
    log = run / ("导航_Real_" + datetime.now().strftime("%Y-%m-%d_%H%M%S"))
    log.mkdir()
    os.environ["ROS_LOG_DIR"] = str(log)
    os.chdir(log)
    procs, handles, cancelled = {}, [], []

    def start(name, *args, interactive=False):
        f = (log / (name + ".log")).open("ab", buffering=0)
        handles.append(f)
        procs[name] = sp.Popen([str(a) for a in args], stdout=f, stderr=sp.STDOUT,
                               stdin=sp.PIPE if interactive else sp.DEVNULL, start_new_session=True)
        return procs[name]

    def cancel(*_):
        cancelled.append(True)

    auto_output = None

    def tick():
        if auto_output is not None:
            for line in auto_output.readlines():
                print(line.rstrip(), flush=True)
        if select.select([sys.stdin], [], [], .15)[0]:
            sys.stdin.readline()
            cancel()
        if cancelled:
            raise InterruptedError("已要求停车")

    def activate(group):
        name = "activate_" + group
        child = start(name, sys.executable, "-Bu", TOOLS / "start_nav_lifecycle_group.py", group)
        started = time.monotonic()
        delay_reported = False
        while child.poll() is None:
            tick()
            if stack.poll() is not None or (not auto_exam and rviz.poll() is not None):
                raise RuntimeError(f"Nav2 或 RViz 已退出，请查看 {log}")
            elapsed = time.monotonic() - started
            if elapsed >= 105:
                raise TimeoutError(f"{group} 激活超时，请查看 {log / (name + '.log')}")
            if elapsed >= 25 and not delay_reported:
                say(f"仍在等待 {group} 节点完成启动，可查看 {log / 'nav2.log'}。")
                delay_reported = True
        if child.returncode:
            raise RuntimeError(f"{group} 激活失败，详见 {log / (name + '.log')} 和 {log / 'nav2.log'}")

    for s in (signal.SIGHUP, signal.SIGINT, signal.SIGTERM):
        signal.signal(s, cancel)
    result, armed, watchdog_fd = 1, False, None
    end_report = {"outcome": "not_completed", "reached": [], "stop_ack": "not_requested"}
    try:
        if opts.recorded:
            marker_args = ["--marker-file", str(original), "--order-file", str(log / "marker-order.json")]
            if opts.exam:
                frozen = log / "teacher-order.csv"
                frozen.write_bytes(order_source.read_bytes())
                marker_args.extend(["--order-csv", str(frozen), "--return-to-start", "--home-file", str(home_file)])
                if opts.rviz_start:
                    marker_args.append("--rviz-start")
                    end_report["trigger"] = "rviz_goal_click"
                end_report.update(order_source=str(order_source), order_snapshot=frozen.name, home_reached=False)
            if opts.shuffle:
                marker_args.append("--shuffle")
                if opts.seed is not None:
                    marker_args.extend(["--seed", str(opts.seed)])
            preview = sp.run([sys.executable, "-Bu", TOOLS / "navigate_saved_markers.py",
                              *marker_args, "--list-only"], text=True, timeout=15, capture_output=True)
            if preview.returncode:
                raise ValueError(preview.stderr.strip() or "路线预检失败")
            order = json.loads(preview.stdout)
            colors = [entry["kind"] for entry in order["order"]]
            if opts.shuffle and opts.seed is None:
                marker_args.extend(["--seed", str(order["seed"])])
            seed_note = f"（随机种子 {order['seed']}）" if opts.shuffle else ""
            if opts.exam:
                say(f"本次考核路线（{total} 个实测地标）：{route_text(colors)}。已保存本次 CSV 快照。")
                if opts.rviz_start:
                    say("手动考核：请在地图上标车的实际位姿。准备好后，终端会提示点 2D Goal Pose 发车。")
                else:
                    say("请将车放回建图时的实体起点并移开手。自动识别车头朝向；不用点 RViz。")
            else:
                say(f"本次将访问 {total} 个实测地标{seed_note}：{' → '.join(colors)}。")
        s = read_pi_status(tick)
        if opts.recorded and not s.get("arm_token_present") and not all(s["pids"].values()):
            say("正在恢复这台真机的驱动和传感器；定位前车轮保持未启用。")
            sp.run(["/bin/bash", str(PI), "prepare"], stdout=sp.DEVNULL, check=True, timeout=30)
            s = read_pi_status(tick)
        if not s.get("fresh") or s.get("arm_token_present") or not all(s["pids"].values()):
            raise RuntimeError("树莓派驱动未就绪或另一任务仍占用车轮。")
        if not opts.recorded and (not meta.get("pi_boot_id") or meta["pi_boot_id"] != (s.get("boot_id") or s.get("pi_boot_id"))):
            raise RuntimeError("树莓派自建图后已重启或其身份无法验证，请重新定位后再导航。")
        say(f"即将加载本次真机地图：{map_yaml.name}。按回车可随时停车。")
        launch_name = "composed_nav2_robot103.launch.py" if opts.recorded else "guarded_nav2.launch.py"
        stack = start("nav2", "ros2", "launch", REAL / "launch" / launch_name, "map:="+str(map_yaml))
        measured = log / "measured_pose.json" if opts.recorded else handoff.parent / "measured_pose.json"
        if opts.recorded and not auto_exam:
            pose_probe = start("localization", sys.executable, "-u", TOOLS / "capture_rviz_localization.py",
                               "--output", measured, "--timeout", "600")
        rviz_config = REAL / "config/真机导航彩色Marker.rviz"
        if opts.exam:
            import yaml
            config = yaml.safe_load(rviz_config.read_text())
            tools = config["Visualization Manager"]["Tools"]
            if opts.rviz_start:
                from rviz_goal_trigger import TRIGGER_TOPIC
                tools.append({"Class": "rviz_default_plugins/SetGoal", "Topic": TRIGGER_TOPIC})
            else:
                tools[:] = [t for t in tools if t["Class"].split("/")[-1] not in ("SetInitialPose", "SetGoal")]
            rviz_config = log / "exam_view.rviz"
            rviz_config.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False))
        try:
            rviz = start("rviz", RVIZ, "-d", rviz_config, "--ros-args",
                         "-r", "__node:=robot103_manual_nav_rviz", "-p", "use_sim_time:=false",
                         *(["-r", "/goal_pose:=" + TRIGGER_TOPIC] if opts.rviz_start else []))
        except OSError:
            if not auto_exam:
                raise
            rviz = None
            say("RViz 未打开；自动考核仍可在本终端运行。")
        start("landmarks", sys.executable, "-u", TOOLS / "vision/show_saved_hardware_markers.py",
              "--marker-file", original)
        if opts.recorded:
            say(f"正在加载 {map_yaml.name} 并激活 AMCL 定位。")
            activate("localization")
        if auto_exam:
            pose_probe = start("localization", sys.executable, "-Bu", TOOLS/"auto_start_localization.py",
                               "--map", map_yaml, "--home-file", home_file, "--output", measured,
                               "--timeout", "0", "--watch-status", log/"auto_start_status.json",
                               "--countdown-request", log/"countdown_request.json", "--arm-request", log/"arm_request.json")
            auto_output = (log / "localization.log").open(errors="replace")
            handles.append(auto_output)
            say("地图服务器已激活。正在等待激光识别建图起点；此时车轮不会启用。")
            while not current_placement(read_placement(log/"auto_start_status.json"), read_placement(measured).get("generation")):
                tick()
                if pose_probe.poll() is not None or stack.poll() is not None:
                    raise RuntimeError(f"自动起点识别或 Nav2 退出，请查看 {log / 'localization.log'} 和 nav2.log")
            say("AMCL 已确认本次起点；正在激活导航节点。")
            activate("navigation")
        elif opts.recorded:
            from PIL import Image
            raster = map_yaml.with_suffix(".pgm")
            with Image.open(raster) as grid:
                width, height = grid.size
            read_event = f"[map_io]: Read map {raster}: {width} X {height} map @"
            rviz_event = f"Trying to create a map of size {width} x {height} using "
            map_read = server_active = prompted = delay_reported = False
            wait_started = time.monotonic()
            with (log / "nav2.log").open(errors="replace") as nav_log, (log / "rviz.log").open(errors="replace") as rviz_log:
                while pose_probe.poll() is None:
                    tick()
                    if stack.poll() is not None or (not auto_exam and rviz.poll() is not None):
                        raise RuntimeError(f"Nav2 或 RViz 已退出，请查看 {log / 'nav2.log'} 和 {log / 'rviz.log'}。")
                    if prompted:
                        continue
                    nav_lines = nav_log.read()
                    map_read |= read_event in nav_lines
                    server_active |= "Server map_server connected with bond." in nav_lines
                    rviz_grid = rviz_event in rviz_log.read()
                    if (map_read and server_active) or rviz_grid:
                        state = f"地图服务器已加载 {map_yaml.name}" if map_read and server_active else f"RViz 已收到 {width} × {height} 格地图"
                        say(f"{state}。看到迷宫后，点 2D Pose Estimate，在车的实际位置按下鼠标，沿车头方向拖动并松开。")
                        prompted = True
                    elif not delay_reported and time.monotonic() - wait_started >= 25:
                        say(f"尚未确认地图已加载，正在等待 Nav2 地图服务就绪。启动详情见 {log / 'nav2.log'}。")
                        delay_reported = True
            if pose_probe.returncode != 0:
                raise RuntimeError("未能确认当前定位："+(log/"localization.log").read_text(errors="replace")[-1000:])
            say("AMCL 已确认本次点选的位姿，正在激活导航节点。")
            activate("navigation")
        else:
            mission = start("mission", sys.executable, "-u", TOOLS / "navigate_saved_markers.py",
                            "--marker-file", handoff, "--pose-file", measured, "--initialize-from-handoff",
                            "--ros-args", "-p", "use_sim_time:=false")
        ready_args = [sys.executable, str(TOOLS / "check_navigation_ready.py"), "--require-navigation-active", "--map", str(map_yaml),
                      "--handoff", str(measured), "--timeout", "45",
                      "--output", str(log / "checks.json")]
        continuity = run / "odom_continuity.json"
        if not opts.recorded and continuity.exists():
            ready_args.extend(["--continuity", str(continuity)])
        def begin_watchdog(mission):
            nonlocal watchdog_fd
            rfd, watchdog_fd = os.pipe()
            try:
                procs["watchdog"] = sp.Popen([sys.executable, str(TOOLS / "nav_terminal_watchdog.py"),
                                              str(rfd), str(stack.pid), str(mission.pid), str(PI)],
                                             pass_fds=(rfd,), start_new_session=True, stdin=sp.DEVNULL)
            finally:
                os.close(rfd)

        event_offset = 0
        if auto_exam:
            mission, check, event_offset, generation = prepare_exam_departure(
                log, measured, start, tick, procs, ready_args, marker_args, begin_watchdog)
            end_report.update(start_generation=generation, start_pose_file=f"start_pose_{generation}.json")
            # This run consumes one placement; returning home cannot restart it.
            stop(pose_probe, wait=1)
            auto_output = None
        else:
            check = start("preflight", *ready_args)
        if opts.recorded and not auto_exam:
            while check.poll() is None:
                tick()
                if stack.poll() is not None:
                    raise RuntimeError("Nav2 已退出，请看 nav2.log")
            if check.returncode:
                raise RuntimeError((log / "preflight.log").read_text(errors="replace")[-1800:])
            say(f"真机正在使用地图 {map_yaml.name}，开始规划地标路线。")
            mission = start("mission", sys.executable, "-u", TOOLS/"navigate_saved_markers.py",
                            *marker_args, "--pose-file", measured)
        deadline = time.monotonic() + 75
        ok_map = opts.recorded
        first_goal = False
        completed = False
        with (log / "mission.log").open(errors="replace") as events:
            events.seek(event_offset)
            while True:
                tick()
                # If the child has exited, its logs are complete. If it exits
                # during this read, drain its final events on the next pass.
                mission_exit = mission.poll()
                for line in events.readlines():
                    if opts.rviz_start and "[RVIZ_TRIGGER_READY]" in line:
                        say("现在可以发车。点 RViz 的 2D Goal Pose，在地图上按下并拖出箭头。点击位置只作启动信号。")
                        deadline = float("inf")
                    if opts.rviz_start and "[RVIZ_TRIGGERED]" in line:
                        say("已收到你的 RViz 点击，开始执行 CSV 路线。")
                        deadline = time.monotonic() + 75
                    if opts.exam and "[HOME_GOAL_ACCEPTED]" in line:
                        say("最后一个地标已完成，正返回建图时的出发位姿。")
                    if "navigating to (" in line:
                        first_goal = True
                        deadline = time.monotonic() + 240
                        say(line.split("]: ")[-1].strip())
                    if "[REACHED]" in line:
                        event = line.split("[REACHED]", 1)[1].strip()
                        end_report["reached"].append(event)
                        say(event)
                    if opts.exam and "[HOME_REACHED]" in line:
                        end_report["home_reached"] = True
                        say("Nav2 已确认回到建图起点。")
                    if opts.exam and "[EXAM_COMPLETE]" in line and end_report["home_reached"]:
                        completed = True
                    elif not opts.exam and ("Visited all four markers in file order" in line or
                            f"Visited all {total} saved markers" in line):
                        completed = True
                if check.poll() is not None and not ok_map:
                    if check.returncode != 0:
                        raise RuntimeError((log / "preflight.log").read_text(errors="replace")[-1800:])
                    ok_map = True
                    say(f"已验证加载的真机地图：{map_yaml.name}，里程计与停车起点相符。")
                if stack.poll() is not None:
                    raise RuntimeError("Nav2 意外退出，请看 nav2.log")
                if mission_exit is not None:
                    if completed and armed and mission_exit == 0:
                        say(f"Nav2 已按 CSV 顺序到达全部 {total} 个地标并返回起点。" if opts.exam
                            else f"Nav2 已依次到达全部 {total} 个地标的接近点。")
                        end_report["outcome"] = "exam_completed_and_home" if opts.exam else "all_goals_succeeded"
                        result = 0
                        break
                    raise RuntimeError("任务未完成："+(log/"mission.log").read_text(errors="replace")[-1100:])
                if time.monotonic() > deadline:
                    raise TimeoutError("导航准备或当前目标超时")
                if ok_map and first_goal and not armed and "arming" not in procs:
                    begin_watchdog(mission)
                    start("arming", "/bin/bash", PI, "arm-nav")
                if "arming" in procs and procs["arming"].poll() is not None and not armed:
                    if procs["arming"].returncode != 0:
                        raise RuntimeError("车轮未放行："+(log/"arming.log").read_text(errors="replace")[-1200:])
                    armed = True
                    say("导航已启用，真机限速驶向目标；按回车停车。")
    except (OSError, ValueError, RuntimeError, sp.SubprocessError, KeyError) as exc:
        end_report["reason"] = str(exc)
        say(str(exc))
    finally:
        # Stop command producers immediately, including when SSH is unavailable.
        for name in ("mission", "activate_navigation", "activate_localization", "nav2"):
            proc = procs.get(name)
            if proc and proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
        if "arming" in procs:
            pipe = procs["arming"].stdin
            if pipe is not None and not pipe.closed:
                pipe.close()
            # Do not race a second ARM write after a stop. The Pi helper bounds its
            # own wait; the already-stopping local smoother can no longer drive.
            try:
                procs["arming"].wait(12)
            except sp.TimeoutExpired:
                stop(procs["arming"], wait=1)
            try:
                feedback = sp.run(["/bin/bash", str(PI), "disarm"], timeout=15, check=True, stdout=sp.PIPE, text=True)
                end_report["stop_ack"] = "zero_command_confirmed"
                end_report["pi_reply"] = feedback.stdout.strip()
                if result == 0:
                    say(f"正常结束：全部 {total} 站已到达，" + ("已回起点，" if opts.exam else "") + "树莓派已确认输出零速度。")
                else:
                    say("任务未完成，树莓派已确认输出零速度。停车前的原因见上一条提示。")
            except (OSError, sp.SubprocessError):
                end_report["stop_ack"] = "unconfirmed"
                say("速度指令已停止，未取得树莓派回执，请现场确认。")
        for name in ("mission", "activate_navigation", "activate_localization", "nav2", "localization", "preflight", "landmarks", "rviz"):
            stop(procs.get(name))
        if watchdog_fd is not None:
            try:
                os.write(watchdog_fd, b"D")
            except BrokenPipeError:
                pass
            os.close(watchdog_fd)
        for f in handles:
            f.close()
        lock.close()
        end_report["finished_local"] = datetime.now().isoformat(timespec="seconds")
        (log / "navigation-result.json").write_text(json.dumps(end_report, ensure_ascii=False, indent=2) + "\n")
        say(f"本次导航日志：{log}")
    return result


if __name__ == "__main__":
    sys.exit(main())
