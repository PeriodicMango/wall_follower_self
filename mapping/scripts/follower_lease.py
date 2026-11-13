"""Stop this session's wall follower if its terminal supervisor disappears."""
import os
import select
import signal
import subprocess
import sys

lease = int(sys.argv[1])
child = subprocess.Popen(sys.argv[2:])
stopping = False


def interrupt(*_):
    global stopping
    stopping = True


for signum in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
    signal.signal(signum, interrupt)
try:
    while not stopping and child.poll() is None:
        if select.select([lease], [], [], .1)[0] and not os.read(lease, 1):
            break
finally:
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(1)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
sys.exit(child.returncode)
