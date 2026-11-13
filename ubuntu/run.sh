#!/usr/bin/env bash
set -e
export TB3_BUNDLE_ROOT="$(cd -- "$(dirname -- "$0")/.." && pwd -P)"
u="$TB3_BUNDLE_ROOT/ubuntu"
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1
usage() { cat <<'USAGE'
From a writable copy of the bundle on Ubuntu 22.04 / ROS 2 Humble:
  ./ubuntu/run.sh preview  [RUN] [--order-csv FILE] [--expect-count 1..4]   offline
  ./ubuntu/run.sh check [nav|mapping]                                     laptop only
  ./ubuntu/run.sh setup [--dedicated|--legacy]                            pair with .103
  ./ubuntu/run.sh status                                                  read Pi status
  ./ubuntu/run.sh navigate [RUN] [--order-csv FILE] [--expect-count 1..4]  manual exam
  ./ubuntu/run.sh build                                                   build follower 2.3
  ./ubuntu/run.sh map                                                     map using 2.3; Enter to save
  ./ubuntu/run.sh camera                                                  view an existing JPEG stream
  ./ubuntu/run.sh stop                                                    disarm the paired robot
Default RUN = the shipped 2026-11-11 15:27 map (3 landmarks); use an explicit run for a new map.
USAGE
}
cmd=${1:-help}
if [[ $# -gt 0 ]]; then shift; fi
case "$cmd" in
 help|-h|--help) usage; exit 0 ;;
 preview) exec /usr/bin/python3 -Bu "$u/navigation.py" preview "$@" ;;
 check|setup|status|navigate|build|map|camera|stop) ;;
 *) usage >&2; exit 2 ;;
esac
if [[ "$(uname -s)" != Linux ]]; then echo 'Use an Ubuntu 22.04 / ROS 2 Humble desktop.' >&2; exit 2; fi
case "$cmd" in
 setup)
    if [[ $# -eq 0 ]]; then set -- --dedicated; fi
    if [[ $# != 1 || ( "$1" != --dedicated && "$1" != --legacy ) ]]; then usage >&2; exit 2; fi
    exec bash "$u/robot.sh" setup "$@" ;;
 status|stop) [[ $# == 0 ]] || { usage >&2; exit 2; }
    if [[ "$cmd" == stop ]]; then cmd=disarm; fi
    exec bash "$u/robot.sh" "$cmd" ;;
esac
if [[ ! -r /opt/ros/humble/setup.bash || ( -n "${ROS_DISTRO:-}" && "$ROS_DISTRO" != humble ) ]]; then
 echo 'Start a clean terminal with Ubuntu 22.04 and /opt/ros/humble installed. See README.' >&2; exit 2
fi
source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=waffle_pi LDS_MODEL=LDS-01
case "$cmd" in
 check) exec bash "$u/check.sh" "$@" ;;
 build) [[ $# == 0 ]] || { usage >&2; exit 2; }; exec bash "$u/mapping-tools/entry.sh" build ;;
esac
case "$cmd" in
 map|camera) [[ $# == 0 ]] || { usage >&2; exit 2; } ;;
esac
if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then echo 'Open an Ubuntu desktop terminal with a display for RViz/camera.' >&2; exit 2; fi
if [[ ! -f "$HOME/.config/tb3-real103/pairing.json" ]]; then echo 'Pair with the physical .103 first: ./ubuntu/run.sh setup' >&2; exit 2; fi
export ROS_DOMAIN_ID=3 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
dds_uri="$(bash "$u/robot.sh" env)"
export CYCLONEDDS_URI="$dds_uri"
case "$cmd" in
 navigate) exec /usr/bin/python3 -Bu "$u/navigation.py" navigate "$@" ;;
 map|camera) exec bash "$u/mapping-tools/entry.sh" "$cmd" ;;
esac
