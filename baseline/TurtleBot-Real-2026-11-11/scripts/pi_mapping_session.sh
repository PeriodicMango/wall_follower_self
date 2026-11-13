#!/usr/bin/env bash
# Run locally on the Mac. No credentials are stored in this script.
set -eu
action=${1:-}
case "$action" in status|prepare|arm|arm-nav|arm-nav-confirmed|disarm) ;; *) echo "Usage: $0 {status|prepare|arm|arm-nav|arm-nav-confirmed|disarm}" >&2; exit 2 ;; esac
[[ $# -eq 1 ]] || exit 2
if [[ $action == disarm && ${TB3_STOP_BOUNDED:-0} != 1 ]]; then
    # Bound master probing, reconnection and the remote stop acknowledgement.
    export TB3_STOP_BOUNDED=1
    exec python3 -c 'import os,signal,subprocess,sys
p=subprocess.Popen(["/bin/bash",sys.argv[1],"disarm"],start_new_session=True)
try: sys.exit(p.wait(timeout=12))
except subprocess.TimeoutExpired:
    os.killpg(p.pid,signal.SIGKILL)
    sys.exit("Disarm acknowledgement timed out; physical stop unverified")' "$0"
fi
script_dir=$(cd "$(dirname "$0")" && pwd)
target=pi@192.168.1.103
run_dir=/home/pi/turtlebot_test_runs/robot103_2026-11-11_143310
old_socket=/tmp/tb3real103.5qMrGwve/s
socket="$HOME/.ssh/tb3-map-103.sock"
key='ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIIPRBm/y+YBtPsRI8VdOPfYvjEU/9wbjbpV1ylxh5Ich'
pin_file=$(mktemp -t tb3-hostkey)
trap 'rm -f "$pin_file"' EXIT
printf '192.168.1.103 %s\n' "$key" > "$pin_file"
opts=(-F /dev/null -o StrictHostKeyChecking=yes -o HostKeyAlgorithms=ssh-ed25519
      -o "UserKnownHostsFile=$pin_file" -o GlobalKnownHostsFile=/dev/null
      -o ConnectTimeout=3 -o ConnectionAttempts=1 -o ServerAliveInterval=2
      -o ServerAliveCountMax=2 -o NumberOfPasswordPrompts=1)
if [[ -S "$old_socket" ]] && ssh "${opts[@]}" -S "$old_socket" -O check "$target" >/dev/null 2>&1; then
    socket=$old_socket
elif ! ssh "${opts[@]}" -S "$socket" -O check "$target" >/dev/null 2>&1; then
    if [[ $action == disarm ]]; then
        # A stop attempt must never wait for a password. A configured key can
        # still reconnect if the previous master was lost.
        opts+=(-o BatchMode=yes)
    fi
    ssh "${opts[@]}" -S "$socket" -o ControlPersist=30m -MNf "$target"
fi
# Verify the same SSH server key even when attaching to an existing master.
# A network outage cannot leave disarm indefinitely pending on the Mac.
remote="test \"\$(cut -d' ' -f1,2 /etc/ssh/ssh_host_ed25519_key.pub)\" = '$key' && test \"\$(cat /sys/class/net/wlan0/address)\" = dc:a6:32:2c:e4:8e || { echo 'TurtleBot host key or wireless MAC mismatch' >&2; exit 1; }; cd '$run_dir' && source robot-env.bash && exec python3 - '$action'"
if [[ $action == arm-nav-confirmed ]]; then
    # Use stdin for the *live* placement reply, rather than consuming it for the script.
    # Pass only hex digits as shell data. An EOF can never release the wheels.
    body=$(python3 -c 'import pathlib,sys;print(pathlib.Path(sys.argv[1]).read_bytes().hex())' "$script_dir/pi_mapping_remote.py")
    remote="${remote%exec python3*}exec python3 -u -c \"exec(bytes.fromhex('$body'))\" '$action'"
    ssh "${opts[@]}" -T -o BatchMode=yes -S "$socket" "$target" "$remote"
else
    ssh "${opts[@]}" -o BatchMode=yes -S "$socket" "$target" "$remote" < "$script_dir/pi_mapping_remote.py"
fi
