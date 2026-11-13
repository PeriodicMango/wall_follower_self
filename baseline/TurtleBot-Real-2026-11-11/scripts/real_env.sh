#!/bin/bash
set -e
base="$(cd "$(dirname "$0")/.." && pwd)"
ws="$(dirname "$base")"
course="$HOME/COMP3431/mac-native-gazebo-11.10.2"
cd "$course"
export PIXI_CACHE_DIR="$course/.cache/pixi"
exec .tools/pixi run --locked bash -c '
  set -e
  source tools/env.sh
  source "$2/TurtleBot-Sim-2026-11-11/teammate_overlay/install/local_setup.bash"
  source "$2/TurtleBot-Test-2026-11-06/restart-robot103/env.sh"
  export TURTLEBOT3_MODEL=waffle_pi PYTHONUNBUFFERED=1
  shift 2
  exec "$@"
' bash "$base" "$ws" "$@"
