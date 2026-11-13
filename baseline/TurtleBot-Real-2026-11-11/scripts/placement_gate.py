"""Local generation-bound files shared by the start verifier, mission and supervisor."""
import json
import os
from pathlib import Path
import tempfile
import time


class PlacementChanged(RuntimeError):
    pass


def read(path):
    try:
        value = json.loads(Path(path).read_text())
    except (FileNotFoundError, ValueError):
        return {}
    return value if isinstance(value, dict) else {}


def publish(path, value):
    path = Path(path)
    with tempfile.NamedTemporaryFile(mode="w", dir=path.parent, delete=False) as stream:
        name = Path(stream.name)
        try:
            json.dump(value, stream, allow_nan=False)
            stream.flush()
            os.replace(name, path)
        finally:
            name.unlink(missing_ok=True)


def current(status, generation, *, max_age=1.0):
    return (type(generation) is int and generation > 0 and status.get("generation") == generation
            and status.get("phase") in {"tracking", "countdown", "ready"}
            and 0 <= time.time() - status.get("updated_unix", 0) < max_age)


def permit(status, generation, token=None):
    return (current(status, generation) and status.get("phase") == "ready"
            and status.get("ready") is True and (token is None or status.get("arm_token") == token))


def wait_for_countdown(status_path, request_path, generation, spin):
    if not current(read(status_path), generation):
        raise PlacementChanged("开始倒计时前起点定位已失效")
    publish(request_path, {"generation": generation})
    while True:
        state = read(status_path)
        if not current(state, generation):
            raise PlacementChanged("倒计时期间起点定位已失效")
        if permit(state, generation):
            return
        spin()
