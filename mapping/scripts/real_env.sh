#!/bin/bash
# Execute a command inside this project's course ROS environment on the Mac.
set -e
root=$(cd "$(dirname "$0")/.." && pwd)
[[ -r "$root/settings.sh" ]] || { echo '先配置 settings.sh 并运行 bash setup_robot.sh。' >&2; exit 1; }
set -a
source "$root/settings.sh"
set +a
export TB3_COURSE="${TB3_COURSE:-$HOME/COMP3431/mac-native-gazebo-11.10.2}"
xml="$root/.robot103/cyclonedds-mac.xml"
[[ -r "$xml" ]] || { echo '缺少本机 DDS 配置，请先运行 bash setup_robot.sh。' >&2; exit 1; }
[[ -x "$TB3_COURSE/.tools/pixi" ]] || { echo '请在 settings.sh 的 TB3_COURSE 填入课程 Mac ROS 环境目录。' >&2; exit 1; }
[[ $# -ge 1 ]] || { echo "Usage: $0 command [args...]" >&2; exit 2; }
cd "$TB3_COURSE"
export PIXI_CACHE_DIR="$TB3_COURSE/.cache/pixi"
exec .tools/pixi run --locked bash -c '
  set -e
  source tools/env.sh
  export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export TURTLEBOT3_MODEL=waffle_pi PYTHONUNBUFFERED=1
  export CYCLONEDDS_URI="file://$1" ROS_LOG_DIR="$2/.robot103/logs"
  export PYTHONPATH="$2/upstream${PYTHONPATH:+:$PYTHONPATH}"
  mkdir -p "$ROS_LOG_DIR"
  cd "$2"
  shift 2
  exec "$@"
' bash "$xml" "$root" "$@"
