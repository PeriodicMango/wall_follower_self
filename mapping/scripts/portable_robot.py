"""Pair this unpacked project with one verified Pi and run its session commands."""
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import re
import shlex
import signal
import subprocess
import sys
import tempfile
import uuid


ROOT = Path(__file__).resolve().parents[1]
STATE = ROOT / ".robot103"
BINDING = STATE / "connection.json"


def identity():
    mac = os.environ.get("TB3_ROBOT_MAC", "").lower()
    if not re.fullmatch(r"(?:[0-9a-f]{2}:){5}[0-9a-f]{2}", mac):
        raise ValueError("在 settings.sh 填入已核对的 TB3_ROBOT_MAC。")
    user = os.environ.get("TB3_SSH_USER", "pi")
    port = int(os.environ.get("TB3_SSH_PORT", 22))
    if not re.fullmatch(r"[a-z_][a-z0-9_-]*", user) or not 1 <= port <= 65535:
        raise ValueError("请核对 TB3_SSH_USER 和 TB3_SSH_PORT。")
    return dict(ip=str(ipaddress.IPv4Address(os.environ.get("TB3_ROBOT_IP", ""))),
                mac=mac, user=user, port=port)


def ssh_args(robot, token, batch=False):
    control = Path.home() / ".ssh" / ("tb3-portable-" + token[:16])
    return ["ssh", "-p", str(robot["port"]), "-S", str(control), "-o", "ControlMaster=auto",
            "-o", "ControlPersist=30m", "-o", "StrictHostKeyChecking=yes", "-o", "ConnectTimeout=3",
            "-o", "ConnectionAttempts=1", "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
            "-o", "NumberOfPasswordPrompts=1", "-o", "BatchMode=" + ("yes" if batch else "no"),
            robot["user"] + "@" + robot["ip"]]


def local_ip(robot_ip):
    route = subprocess.check_output(["/sbin/route", "-n", "get", robot_ip], text=True)
    match = re.search(r"^\s*interface:\s*(\S+)", route, re.M)
    if not match:
        raise ValueError("找不到到真机的 Wi-Fi 路由。")
    ip = subprocess.check_output(["/usr/sbin/ipconfig", "getifaddr", match[1]], text=True).strip()
    return str(ipaddress.IPv4Address(ip))


def xml(interface, mac_ip, robot_ip, tag):
    return f"""<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="3">
  <General><Interfaces><NetworkInterface address="{interface}"/></Interfaces>
    <AllowMulticast>false</AllowMulticast><MulticastRecvNetworkInterfaceAddresses>none</MulticastRecvNetworkInterfaceAddresses></General>
  <Discovery><Tag>{tag}</Tag><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
    <Peers><Peer Address="{mac_ip}"/><Peer Address="{robot_ip}"/></Peers></Discovery>
</Domain></CycloneDDS>
"""


def write_private(path, data):
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as tmp:
        tmp.write(data)
    os.replace(tmp.name, path)


# Executed once on the Pi via SSH/Python. Install an immutable file set in a
# new private directory. Existing drives, ARM files and processes are untouched.
DEPLOY = """
import json, os, pathlib, shutil, sys, tempfile
p = json.load(sys.stdin)
home = pathlib.Path.home()
if pathlib.Path('/sys/class/net/wlan0/address').read_text().strip().lower() != p['mac']:
    sys.exit('机器人 wlan0 硬件地址与 settings.sh 不符，未部署。')
for item in p['setups']:
    if not pathlib.Path(item).expanduser().is_file():
        sys.exit('Pi 缺少 ROS 环境脚本 ' + item)
dest = home / p['directory']
files = p['files']
if dest.exists() or dest.is_symlink():
    if dest.is_symlink() or not dest.is_dir() or dest.stat().st_uid != os.getuid():
        sys.exit('已有同名目录不属于本次部署。')
    if not all((dest/n).is_file() and not (dest/n).is_symlink() and (dest/n).read_text() == v
               for n,v in files.items()):
        sys.exit('同名目录内容不同，未覆盖。')
else:
    stage = pathlib.Path(tempfile.mkdtemp(prefix='.tb3-stage-', dir=home))
    try:
        for name, data in files.items():
            (stage/name).write_text(data)
        stage.rename(dest)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
print('Pi session directory:',dest)
"""


def setup():
    robot = identity()
    STATE.mkdir(mode=0o700, exist_ok=True)
    with (STATE / "setup.lock").open("a") as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        if BINDING.is_file():
            old = json.loads(BINDING.read_text())
            if old["robot"] != robot:
                raise ValueError("这个解压目录已绑定另一台设备或 SSH 端口。新配对请使用另一份干净解压目录。")
            token = old["token"]
        else:
            token = uuid.uuid4().hex
        mac_ip = local_ip(robot["ip"])
        tag = "tb3portable_" + token
        ros_setups = [os.environ.get("TB3_PI_ROS_SETUP", "/opt/ros/humble/setup.bash"),
                      os.environ.get("TB3_PI_WORKSPACE_SETUP", "~/turtlebot3_ws/install/setup.bash")]
        def shell_path(p):
            if p.startswith("~/"):
                return '"$HOME"/' + shlex.quote(p[2:])
            if not p.startswith("/"):
                raise ValueError("Pi 环境脚本需使用绝对路径或 ~/ 开头。")
            return shlex.quote(p)
        files = {n: (ROOT / "scripts" / n).read_text() for n in ("robot_velocity_guard.py", "pi_mapping_remote.py")}
        files["pi_camera_jpeg.py"] = (ROOT / "scripts/vision/pi_camera_jpeg.py").read_text()
        files["cyclonedds-robot.xml"] = xml(robot["ip"], mac_ip, robot["ip"], tag)
        files["robot-env.bash"] = "\n".join([
            *(f"source {shell_path(p)} || return 1" for p in ros_setups),
            "export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp",
            "export TURTLEBOT3_MODEL=waffle_pi LDS_MODEL=LDS-01",
            'export TB3_SESSION_DIR="$PWD" CYCLONEDDS_URI="file://$PWD/cyclonedds-robot.xml"', "",
        ])
        version = hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()[:12]
        directory = f"turtlebot23_{token}_{version}"
        files[".portable-owner"] = token + "\n"
        files[".portable-files.sha256"] = "".join(
            hashlib.sha256(data.encode()).hexdigest() + "  " + name + "\n" for name, data in sorted(files.items()))
        payload = dict(directory=directory, files=files, mac=robot["mac"], setups=ros_setups)
        subprocess.run(ssh_args(robot, token) + [shlex.join(["python3", "-c", DEPLOY])],
                       input=json.dumps(payload), text=True, check=True)
        write_private(STATE / "cyclonedds-mac.xml", xml(mac_ip, mac_ip, robot["ip"], tag))
        write_private(BINDING, json.dumps(dict(robot=robot, token=token, directory=directory, mac_ip=mac_ip)) + "\n")
        print("配对完成，Mac 地址", mac_ip, "。运行建图命令时才启动驱动。")


def command(action):
    robot = identity()
    conf = json.loads(BINDING.read_text())
    token, directory = conf["token"], conf["directory"]
    if (conf["robot"] != robot or not re.fullmatch(r"[0-9a-f]{32}", token) or
            not re.fullmatch("turtlebot23_" + token + r"_[0-9a-f]{12}", directory)):
        raise ValueError("当前设置与已配对的连接不一致。")
    if action in ("prepare", "arm", "arm-nav") and local_ip(robot["ip"]) != conf["mac_ip"]:
        raise ValueError("Mac 无线地址已更改；请在停止现有任务后重新运行 setup_robot.sh。")
    script = (
        'set -e; cd "$HOME/$1"; '
        '[[ "$(cat /sys/class/net/wlan0/address)" == "$2" && "$(cat .portable-owner)" == "$3" ]] || '
        '{ echo "真机身份或会话目录不符" >&2; exit 1; }; '
        'source robot-env.bash; '
        'if [[ "$4" == prepare || "$4" == arm || "$4" == arm-nav ]]; then '
        'sha256sum --check --status .portable-files.sha256; '
        'exec flock -n "$HOME/.turtlebot-portable-driver.lock" python3 ./pi_mapping_remote.py "$4"; fi; '
        'exec python3 ./pi_mapping_remote.py "$4"'
    )
    remote = shlex.join(["bash", "-c", script, "bash", directory, robot["mac"], token, action])
    proc = subprocess.Popen(ssh_args(robot, token, batch=action == "disarm") + [remote],
                            start_new_session=(action == "disarm"))
    try:
        return proc.wait(timeout=12 if action == "disarm" else None)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
        raise RuntimeError("停车回执超时，请现场确认机器人是否停止。") from None


if __name__ == "__main__":
    try:
        action, = sys.argv[1:]
        if action == "setup":
            setup()
        elif action in ("status", "prepare", "arm", "arm-nav", "disarm"):
            sys.exit(command(action))
        else:
            raise ValueError("未知的树莓派会话操作。")
    except (OSError, ValueError, KeyError, RuntimeError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
