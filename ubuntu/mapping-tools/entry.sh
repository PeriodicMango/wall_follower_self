#!/usr/bin/env bash
# Called by the bundle's Ubuntu runner after it sources the ROS environment.
set -eu
bundle="${TB3_BUNDLE_ROOT:?Start this command through ubuntu/run.sh}"
tools_dir="$bundle/ubuntu/mapping-tools"
mapping="$bundle/mapping"
variant="$mapping/local_variants/wall_follower_2_3"
if [[ "$(uname -s)" != Linux || "${ROS_DISTRO:-}" != humble ]]; then
    echo "Mapping requires Ubuntu 22.04 with ROS 2 Humble sourced." >&2
    exit 2
fi
if [[ $# -ne 1 ]]; then
    echo "Usage: entry.sh {build|map|camera}" >&2
    exit 2
fi
export PYTHONPATH="$mapping/upstream${PYTHONPATH:+:$PYTHONPATH}"
case "$1" in
    build)
        cmake -S "$variant" -B "$variant/build" -DCMAKE_BUILD_TYPE=Release -DPython3_EXECUTABLE=/usr/bin/python3
        cmake --build "$variant/build" --parallel 2
        test -x "$variant/build/wall_follower_2_3"
        ;;
    map|camera)
        if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
            echo "Open a terminal inside the Ubuntu desktop so RViz and the camera can display." >&2
            exit 2
        fi
        if [[ "$1" == camera ]]; then
            exec /usr/bin/python3 -Bu "$mapping/scripts/vision/camera_preview.py"
        fi
        exec /usr/bin/python3 -Bu "$tools_dir/manual_mapping.py"
        ;;
    *)
        echo "Usage: entry.sh {build|map|camera}" >&2
        exit 2
        ;;
esac
