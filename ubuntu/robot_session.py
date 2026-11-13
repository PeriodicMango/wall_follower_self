"""Pair one Ubuntu laptop with the verified .103, then relay the archived Pi protocol."""
import fcntl
import hashlib
import ipaddress
import json
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import tempfile
import uuid

PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "baseline/TurtleBot-Real-2026-11-11/scripts"
STATE = Path.home() / ".config/tb3-real103"
BINDING = STATE / "pairing.json"
HOST_XML = STATE / "cyclonedds-mac.xml"
IP = "192.168.1.103"
MAC = "dc:a6:32:2c:e4:8e"
HOST_KEY = "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIIPRBm/y+YBtPsRI8VdOPfYvjEU/9wbjbpV1ylxh5Ich"
LEGACY_DIR = "/home/pi/turtlebot_test_runs/robot103_2026-11-11_143310"
LEGACY_TAG = "tbtest-01a1af88-robot103"
ACTIONS = {"status", "prepare", "arm", "arm-nav", "arm-nav-confirmed", "disarm"}


def write_local(path, value):
    if path.is_file() and path.read_text() == value:
        return
    with tempfile.NamedTemporaryFile(mode="w", dir=STATE, delete=False) as tmp:
        tmp.write(value)
    os.replace(tmp.name, path)


def route_ip():
    entries = json.loads(subprocess.check_output(["ip", "-j", "-4", "route", "get", IP], text=True))
    address = ipaddress.IPv4Address(entries[0]["prefsrc"])
    if address.is_loopback or str(address) == IP:
        raise ValueError("Invalid Wi-Fi route to .103. Connect to the robot LAN.")
    return str(address)


def xml(interface, laptop, tag):
    return f"""<CycloneDDS xmlns="https://cdds.io/config"><Domain Id="3">
  <General><Interfaces><NetworkInterface address="{interface}"/></Interfaces>
    <AllowMulticast>false</AllowMulticast><MulticastRecvNetworkInterfaceAddresses>none</MulticastRecvNetworkInterfaceAddresses></General>
  <Discovery><Tag>{tag}</Tag><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
    <Peers><Peer Address="{laptop}"/><Peer Address="{IP}"/></Peers></Discovery>
</Domain></CycloneDDS>
"""


def ssh_cmd(remote, batch=False):
    known_hosts = STATE / "known_hosts_103"
    write_local(known_hosts, IP + " " + HOST_KEY + "\n")
    # Keep the Unix socket path short on laptops with long usernames.
    socket = f"/tmp/tb3-nav103-{os.getuid()}"
    control_dir = Path(socket)
    control_dir.mkdir(mode=0o700, exist_ok=True)
    if control_dir.is_symlink() or control_dir.stat().st_uid != os.getuid() or control_dir.stat().st_mode & 0o077:
        raise RuntimeError("Unsafe SSH control directory " + socket)
    return ["ssh", "-F", "/dev/null", "-T", "-S", socket + "/s", "-o", "ControlMaster=auto", "-o", "ControlPersist=30m",
            "-o", "StrictHostKeyChecking=yes", "-o", "HostKeyAlgorithms=ssh-ed25519",
            "-o", "UserKnownHostsFile=" + str(known_hosts), "-o", "GlobalKnownHostsFile=/dev/null",
            "-o", "NumberOfPasswordPrompts=1", "-o", "ConnectTimeout=3", "-o", "ConnectionAttempts=1",
            "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
            "-o", "BatchMode=" + ("yes" if batch else "no"), "pi@" + IP, remote]


VERIFY = (
    'test "$(cut -d\' \' -f1,2 /etc/ssh/ssh_host_ed25519_key.pub)" = ' + shlex.quote(HOST_KEY)
    + ' && test "$(cat /sys/class/net/wlan0/address)" = ' + shlex.quote(MAC)
    + ' || { echo ".103 robot identity mismatch" >&2; exit 1; }; '
)


def bash_remote(script, *params):
    return shlex.join(["bash", "-c", VERIFY + "set -e; " + script, "tb3", *params])


# Install into a fresh private directory per tag and laptop IP. Never overwrite
# an existing Pi session, its ARM token or its live Cyclone configuration.
INSTALL = """
import json, os, pathlib, shutil, sys, tempfile
p = json.load(sys.stdin)
base = pathlib.Path.home() / 'turtlebot_test_runs'
base.mkdir(exist_ok=True)
dest = base / p['dir']
for script in ('/opt/ros/humble/setup.bash','/home/pi/turtlebot3_ws/install/setup.bash'):
    if not pathlib.Path(script).is_file():
        sys.exit('Missing Pi ROS setup: ' + script)
if dest.exists() or dest.is_symlink():
    if dest.is_symlink() or not dest.is_dir() or dest.stat().st_uid != os.getuid():
        sys.exit('Pi directory identity mismatch; nothing changed.')
    for n,content in p['files'].items():
        file = dest / n
        if file.is_symlink() or not file.is_file() or file.read_text() != content:
            sys.exit('Pi directory exists with different contents; nothing overwritten.')
else:
    stage = pathlib.Path(tempfile.mkdtemp(dir=base,prefix='.ubuntu-stage-'))
    try:
        for n,content in p['files'].items():
            (stage/n).write_text(content)
        stage.rename(dest)
    finally:
        if stage.exists():
            shutil.rmtree(stage)
"""

PI_ENV = """source /opt/ros/humble/setup.bash || return 1
source /home/pi/turtlebot3_ws/install/setup.bash || return 1
export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export TURTLEBOT3_MODEL=waffle_pi LDS_MODEL=LDS-01
export TB3_SESSION_DIR="$PWD" CYCLONEDDS_URI="file://$PWD/cyclonedds-robot.xml"
"""


def dedicated(laptop, old):
    token = old.get("token") or uuid.uuid4().hex
    tag = "tb3ubuntu_" + token
    files = {"robot_velocity_guard.py": (SCRIPTS / "robot_velocity_guard.py").read_text(),
             "pi_camera_jpeg.py": (SCRIPTS / "vision/pi_camera_jpeg.py").read_text(),
             "cyclonedds-robot.xml": xml(IP, laptop, tag), "robot-env.bash": PI_ENV}
    digests = "".join(hashlib.sha256(text.encode()).hexdigest() + "  " + name + "\n"
                      for name, text in sorted(files.items()))
    files[".installed.sha256"] = digests
    name = "ubuntu_exam_" + token + "_" + hashlib.sha256(digests.encode()).hexdigest()[:12]
    remote = bash_remote('exec python3 -c "$1"', INSTALL)
    subprocess.run(ssh_cmd(remote), input=json.dumps({"dir": name, "files": files}), text=True, check=True)
    return {"mode": "dedicated", "laptop": laptop, "token": token, "tag": tag,
            "dir": "/home/pi/turtlebot_test_runs/" + name}


def legacy(laptop):
    probe = ('cd "$1"; source robot-env.bash; test "$ROS_DOMAIN_ID" = 3; '
             'test -r robot_velocity_guard.py; test -r pi_camera_jpeg.py; '
             'python3 -c "$2" "${CYCLONEDDS_URI#file://}" "$3"')
    inspect = """import sys,xml.etree.ElementTree as E
root=E.parse(sys.argv[1]).getroot()
tags=[n.text for n in root.iter() if n.tag.rsplit('}',1)[-1]=='Tag']
if sys.argv[2] not in tags: sys.exit('Saved Pi session uses a different DDS tag.')
"""
    subprocess.run(ssh_cmd(bash_remote(probe, LEGACY_DIR, inspect, LEGACY_TAG)), check=True)
    return {"mode": "legacy", "laptop": laptop, "tag": LEGACY_TAG, "dir": LEGACY_DIR}


def setup(mode):
    laptop = route_ip()
    old = json.loads(BINDING.read_text()) if BINDING.exists() else {}
    changing = bool(old and (old["mode"] != mode or old["laptop"] != laptop))
    if changing:
        # Refuse a binding switch until the previous session's hardware processes
        # have exited. The old disarm route stays available before a coordinated reboot.
        remote = bash_remote('cd "$1"; source robot-env.bash; exec python3 - status', old["dir"])
        s = subprocess.run(ssh_cmd(remote), input=(SCRIPTS / "pi_mapping_remote.py").read_text(),
                           capture_output=True, text=True)
        if s.returncode:
            raise RuntimeError("Previous session status check failed. " + s.stderr.strip())
        status = json.loads(s.stdout)
        if any(status["pids"].values()):
            raise RuntimeError("Finish the previous operator's run, then arrange a cold Pi restart before switching pairing.")
    conf = dedicated(laptop, old) if mode == "dedicated" else legacy(laptop)
    write_local(HOST_XML, xml(laptop, laptop, conf["tag"]))
    write_local(BINDING, json.dumps(conf, indent=2) + "\n")
    print(f"Paired {laptop} to .103 ({mode}), Pi directory {conf['dir']}. Wheels were not enabled.")


def relay(action, conf):
    if action in {"prepare", "arm", "arm-nav", "arm-nav-confirmed"} and route_ip() != conf["laptop"]:
        raise RuntimeError("Laptop address changed. Stop the session and repeat pairing.")
    src = (SCRIPTS / "pi_mapping_remote.py").read_bytes()
    pre = 'cd "$1"; source robot-env.bash; '
    if conf["mode"] == "dedicated" and action in {"prepare", "arm", "arm-nav", "arm-nav-confirmed"}:
        pre += 'sha256sum --check --status .installed.sha256; '
    if action in {"prepare", "arm", "arm-nav", "arm-nav-confirmed"}:
        pre += 'exec 9>"$HOME/.tb3-ubuntu-103.lock"; flock -n 9 || { echo "A Pi setup/arming operation is in progress" >&2; exit 1; }; '
    if action == "arm-nav-confirmed":
        remote = bash_remote(pre + 'exec python3 -u -c "$3" "$2"', conf["dir"], action,
                             'exec(bytes.fromhex("' + src.hex() + '"))')
        return subprocess.run(ssh_cmd(remote)).returncode
    cmd = ssh_cmd(bash_remote(pre + 'exec python3 - "$2"', conf["dir"], action), batch=action == "disarm")
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE, start_new_session=(action == "disarm"))
    try:
        proc.communicate(src, timeout=12 if action == "disarm" else None)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.communicate()
        raise RuntimeError("Disarm acknowledgement timed out. Check the physical robot.") from None
    return proc.returncode


def main():
    args = sys.argv[1:]
    STATE.mkdir(mode=0o700, parents=True, exist_ok=True)
    if args in (["setup"], ["setup", "--legacy"], ["setup", "--dedicated"]):
        with (STATE / "setup.lock").open("a") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            setup("dedicated" if "--dedicated" in args else "legacy")
        return 0
    if len(args) != 1 or args[0] not in ACTIONS | {"env"}:
        raise ValueError("Usage: robot.sh setup [--legacy|--dedicated] | env | status | prepare | arm | arm-nav | arm-nav-confirmed | disarm")
    if not BINDING.is_file() and args[0] != "env":
        raise ValueError("First run bash ubuntu/robot.sh setup (or setup --dedicated).")
    if args[0] == "env":
        laptop = route_ip()
        conf = json.loads(BINDING.read_text()) if BINDING.exists() else {"tag": LEGACY_TAG, "laptop": laptop}
        if conf["laptop"] != laptop:
            raise ValueError("Laptop address changed. Finish the previous session and run setup again.")
        write_local(HOST_XML, xml(laptop, laptop, conf["tag"]))
        print(HOST_XML.as_uri())
        return 0
    return relay(args[0], json.loads(BINDING.read_text()))


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, RuntimeError, KeyError, ValueError, subprocess.CalledProcessError) as exc:
        sys.exit(str(exc))
