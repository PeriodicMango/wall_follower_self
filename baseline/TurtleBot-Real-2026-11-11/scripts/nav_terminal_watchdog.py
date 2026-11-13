"""If the user's navigation terminal dies, stop only its navigation producer groups."""
import os
import signal
import subprocess
import sys
import time

read_fd, stack_pid, mission_pid = map(int, sys.argv[1:4])
if os.read(read_fd, 1) != b"D":
    for pid in (mission_pid, stack_pid):
        try:
            os.killpg(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    # During first-goal arming the Pi lock may still be busy. Command producers
    # are already shutting down; retry the exact session's stop acknowledgement.
    for _ in range(3):
        try:
            if subprocess.run(["/bin/bash", sys.argv[4], "disarm"], timeout=15).returncode == 0:
                break
        except (OSError, subprocess.TimeoutExpired):
            pass
        time.sleep(1)
