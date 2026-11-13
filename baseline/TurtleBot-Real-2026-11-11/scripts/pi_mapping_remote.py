"""Companion to pi_mapping_session.sh; executed on the pinned Pi via SSH stdin."""
import fcntl
import json
import math
import os
import subprocess
import sys
import time
import uuid
from pathlib import Path

RUN = Path(os.environ.get("TB3_SESSION_DIR", os.getcwd())).resolve()
GUARD = RUN / "robot_velocity_guard.py"
ARM = RUN / "ARM"
STATUS = RUN / "status.json"
STOP_REQUEST = RUN / ".mapping-last-disarm"
DISARMED = {"awaiting new ARM token", "ARM removed; new token required"}
COMMANDS = {
    "guard": ["python3", "-u", str(GUARD), "--continuous", "--arm-file", str(ARM), "--status-file", str(STATUS)],
    "bringup": ["ros2", "launch", "turtlebot3_bringup", "robot.launch.py"],
    "camera": ["ros2", "launch", "turtlebot3_bringup", "camera.launch.py", "use_image_view:=false"],
    "pi_camera_jpeg": ["python3", "-u", "pi_camera_jpeg.py"],
}


def service_pids():
    matched = {name: [] for name in COMMANDS}
    hardware_processes = {"bringup": [], "camera": []}
    for path in Path("/proc").glob("[0-9]*/cmdline"):
        try:
            argv = path.read_bytes().decode().strip("\0").split("\0")
            names = [Path(arg).name for arg in argv]
            name = None
            if names[0] == "turtlebot3_ros":
                hardware_processes["bringup"].append(int(path.parent.name))
            if names[0].startswith("component_container") and any("camera_container" in v for v in argv):
                hardware_processes["camera"].append(int(path.parent.name))
            # Recognize the old script as well: prepare must not start a
            # second /cmd_vel publisher while the old guard is still running.
            guard_scripts = {"guard.py", GUARD.name}
            guard_script = next((i for i,n in enumerate(names) if n in guard_scripts), None)
            if guard_script is not None and (names[0].startswith("python") or guard_script == 0):
                name = "guard"
                cwd = (path.parent / "cwd").resolve()
                script_path = (cwd / argv[guard_script]).resolve()
                if script_path != GUARD.resolve():
                    raise RuntimeError(f"Unexpected velocity guard PID {path.parent.name} at {script_path}; "
                                       f"expected {GUARD}. Complete the guard replacement before prepare or arm.")
                if ("--continuous" not in argv or
                        any(flag not in argv or argv.index(flag) + 1 >= len(argv)
                            or (cwd / argv[argv.index(flag) + 1]).resolve() != expected.resolve()
                            for flag, expected in (("--arm-file", ARM), ("--status-file", STATUS)))):
                    raise RuntimeError(f"Conflicting guard process {path.parent.name}: {argv}")
            elif "pi_camera_jpeg.py" in names:
                name = "pi_camera_jpeg"
                script = argv[names.index("pi_camera_jpeg.py")]
                if ((path.parent / "cwd").resolve() / script).resolve() != RUN / "pi_camera_jpeg.py":
                    raise RuntimeError(f"Conflicting camera bridge {path.parent.name}: {argv}")
            elif "ros2" in names:
                args = argv[names.index("ros2") + 1:]
                if args[:2] == ["launch", "turtlebot3_bringup"] and len(args) >= 3:
                    name = {"robot.launch.py": "bringup", "camera.launch.py": "camera"}.get(args[2])
            if name:
                env = (path.parent / "environ").read_bytes().split(b"\0")
                for key in ("ROS_DOMAIN_ID", "CYCLONEDDS_URI"):
                    if f"{key}={os.environ[key]}".encode() not in env:
                        raise RuntimeError(f"{name} PID {path.parent.name} uses a different {key}")
                matched[name].append(int(path.parent.name))
        except (FileNotFoundError, ProcessLookupError, UnicodeDecodeError):
            continue
    for name, pids in matched.items():
        if len(pids) > 1:
            raise RuntimeError(f"Multiple {name} processes {pids}")
        children = hardware_processes.get(name, [])
        if len(children) > 1 or (children and not pids):
            raise RuntimeError(f"Orphan or duplicate {name}: launch={pids}, hardware={children}")
    return matched


def latest():
    try:
        return json.loads(STATUS.read_text())
    except (FileNotFoundError, ValueError):
        return {}


def fresh(data):
    return 0 <= time.time() - data.get("updated_at_unix", 0) < 1.8


def wait_for(predicate, after, failure, timeout=6):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        data = latest()
        if fresh(data) and data["updated_at_unix"] > after and predicate(data):
            return data
        time.sleep(0.12)
    raise RuntimeError(f"{failure}. Latest status: {latest()}")


def held(data):
    return (data.get("state") == "holding_zero" and bool(set(data.get("reasons", ())) & DISARMED)
            and data.get("output") == {"linear_x": 0.0, "angular_z": 0.0})


def healthy(data):
    ages = data.get("age_seconds", {})
    reasons = set(data.get("reasons", ()))
    scan_min = data.get("minimum_valid_range_m")
    return (not reasons - DISARMED and type(scan_min) in (int, float) and math.isfinite(scan_min)
            and scan_min >= 0 and all(isinstance(ages.get(k), (int, float)) and
                                         0 <= ages[k] < limit and data.get("counts", {}).get(k, 0) > 0
                                         for k, limit in (("scan", .6), ("odom", .6), ("command", .3))))


def clear_token():
    after = time.time()
    ARM.write_text("")
    return after


def valid_command_graph(graph, sinks, navigation):
    if (graph["/cmd_vel"] != ["turtlebot_test_velocity_guard"] or
            len(graph["/scan"]) != 1 or len(graph["/odom"]) != 1 or "turtlebot3_node" not in sinks):
        return False
    source = graph["/turtlebot_test/wall_cmd_vel"]
    if navigation:
        raw = graph["/turtlebot_test/nav_cmd_vel"]
        return (source == ["velocity_smoother"] and raw.count("controller_server") == 1
                and set(raw) == {"controller_server", "behavior_server"})
    return source in (["wall_follower"], ["wall_follower_node"])


def validate_graph(navigation):
    import rclpy
    rclpy.init()
    node = rclpy.create_node("pi_mapping_endpoint_check", enable_rosout=False, start_parameter_services=False)
    try:
        def endpoint_name(endpoint):
            return "/".join(part for part in (endpoint.node_namespace.strip("/"), endpoint.node_name) if part)

        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.15)
            def publishers(topic):
                return [endpoint_name(p) for p in node.get_publishers_info_by_topic(topic)]
            graph = {topic: publishers(topic) for topic in (
                "/cmd_vel", "/turtlebot_test/wall_cmd_vel", "/turtlebot_test/nav_cmd_vel", "/scan", "/odom")}
            sinks = [endpoint_name(p) for p in node.get_subscriptions_info_by_topic("/cmd_vel")]
            if valid_command_graph(graph, sinks, navigation):
                return
        raise RuntimeError(f"Guarded command path or sensor publisher conflict: {graph}; base endpoints: {sinks}")
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


def prepare():
    services = service_pids()
    for path in (GUARD, RUN / "pi_camera_jpeg.py"):
        if not path.is_file():
            raise RuntimeError(f"Missing installed component {path}")
    if services["guard"]:
        if ARM.is_file() and ARM.read_text().strip():
            raise RuntimeError("ARM is in use by an existing guard; this session did not change it")
        after = time.time()
        wait_for(held, after, "Existing guard is not confirmed disarmed; this session did not change it")
    else:
        # Boot may leave a persisted token, but service_pids established that
        # no guard owns it. Restart the missing guard in a disarmed state.
        after = clear_token()
    for name, argv in COMMANDS.items():
        if not services[name]:
            with (RUN / f"{name}.log").open("ab") as log:
                proc = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                        cwd=RUN, start_new_session=True, close_fds=True)
            (RUN / f"{name}.pid").write_text(str(proc.pid) + "\n")
    data = wait_for(held, after, "Guard failed to acknowledge disarmed state", timeout=10)
    services = service_pids()
    if not all(services.values()):
        raise RuntimeError(f"A service failed to start: {services}. See {RUN}/*.log")
    return {"pids": services, "guard": data, "boot_id": boot_id()}


def final_placement_confirmation():
    """Ask for one new scan *after* the potentially slow graph/health probes."""
    import select
    challenge = uuid.uuid4().hex
    started = time.monotonic()
    print(json.dumps({"placement_challenge": challenge}), flush=True)
    if not select.select([sys.stdin], [], [], 2.0)[0]:
        raise RuntimeError("Timed out waiting for the fresh start-placement reply; ARM not written")
    reply = json.loads(sys.stdin.readline(512))
    if (reply.get("challenge") != challenge or time.monotonic() - started > 2.
            or not 0 < reply.get("valid_until", 0) - time.time() <= 1.5):
        raise RuntimeError("Expired or invalid start placement; check Pi/Mac clocks; ARM not written")
    return reply["valid_until"]


def arm(navigation=False, confirm_placement=False):
    def stop_generation():
        return STOP_REQUEST.read_text() if STOP_REQUEST.exists() else ""

    previous_stop = stop_generation()
    if not all(service_pids().values()):
        raise RuntimeError("A required Pi component is missing; run prepare")
    validate_graph(navigation)
    data = wait_for(healthy, time.time(), "Inputs are missing, stale or invalid, or the guard is offline")
    requested_mode = "navigation" if navigation else "mapping"
    if navigation and (data.get("guard_schema_version") != 2 or data.get("navigation_forward_limit_m_s") != 0.13):
        raise RuntimeError("Installed velocity guard does not support the 0.13 m/s navigation mode")
    if data["state"] == "armed_ready" and ARM.read_text().strip():
        if confirm_placement:
            raise RuntimeError("Wheels already armed by another request")
        if data.get("mode", "mapping") != requested_mode:
            raise RuntimeError("Guard is armed for a different command source; disarm first")
        return data
    if not held(data) or ARM.read_text().strip():
        raise RuntimeError("ARM and guard state disagree; run disarm")
    valid_until = math.inf
    if confirm_placement:
        valid_until = final_placement_confirmation()
        live = latest()
        if not fresh(live) or not healthy(live) or not held(live) or ARM.read_text().strip():
            raise RuntimeError("Guard inputs changed during placement confirmation; ARM not written")
    temp = RUN / f".ARM.{os.getpid()}.tmp"
    token = uuid.uuid4().hex
    temp.write_text(("navigation:" if navigation else "") + token + "\n")
    after = time.time()
    if stop_generation() != previous_stop or time.time() >= valid_until:
        temp.unlink()
        raise RuntimeError("A concurrent disarm canceled this arm request")
    temp.replace(ARM)
    try:
        def confirmed(data):
            if stop_generation() != previous_stop:
                raise RuntimeError("A concurrent disarm canceled this arm request")
            mode_ok = (data.get("mode", "mapping") == requested_mode and
                       (not navigation or data.get("forward_limit_m_s") == 0.13))
            return data.get("state") == "armed_ready" and healthy(data) and mode_ok

        return wait_for(confirmed, after, "Guard did not confirm healthy arming", timeout=3)
    except BaseException:
        clear_token()
        raise


def main():
    if Path.cwd() != RUN or os.environ.get("ROS_DOMAIN_ID") != "3":
        raise RuntimeError("Expected the existing Nov 11 hardware run and ROS Domain 3")
    uri = os.environ.get("CYCLONEDDS_URI", "")
    if not uri.startswith("file://") or not Path(uri[7:]).is_file():
        raise RuntimeError("The existing Pi CycloneDDS hardware configuration is missing")
    action = sys.argv[1]
    if action == "status":
        return {"pids": service_pids(), "arm_token_present": bool(ARM.is_file() and ARM.read_text().strip()),
                "fresh": fresh(latest()), "guard": latest(), "boot_id": boot_id()}
    if action == "disarm":
        # Stop takes precedence even if arm is still discovering ROS endpoints.
        STOP_REQUEST.write_text(uuid.uuid4().hex)
        after = clear_token()
        return wait_for(held, after, "ARM was cleared but a fresh stop acknowledgement is missing")
    with (RUN / ".mapping-session.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        if action == "prepare":
            return prepare()
        if action in ("arm", "arm-nav", "arm-nav-confirmed"):
            return arm(navigation=action != "arm", confirm_placement=action == "arm-nav-confirmed")
        raise ValueError(action)


def boot_id():
    return Path("/proc/sys/kernel/random/boot_id").read_text().strip()


if __name__ == "__main__":
    try:
        print(json.dumps(main(), ensure_ascii=False), flush=True)
    except (OSError, ValueError, RuntimeError) as exc:
        sys.exit(str(exc))
