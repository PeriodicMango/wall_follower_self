#!/bin/bash
set -e
root=$(cd "$(dirname "$0")/.." && pwd)
[[ -r "$root/settings.sh" ]] || { echo '先配置 settings.sh 并运行 bash setup_robot.sh。' >&2; exit 1; }
case "${1:-}" in status|prepare|arm|arm-nav|disarm) ;; *) echo "Usage: $0 {status|prepare|arm|arm-nav|disarm}" >&2; exit 2 ;; esac
[[ $# -eq 1 ]] || exit 2
set -a
source "$root/settings.sh"
set +a
exec python3 "$root/scripts/portable_robot.py" "$1"
