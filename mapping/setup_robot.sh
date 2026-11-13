#!/bin/bash
set -e
root=$(cd "$(dirname "$0")" && pwd)
[[ -r "$root/settings.sh" ]] || { echo '先复制 settings.example.sh 为 settings.sh，并填写真机 IP 和无线 MAC。' >&2; exit 1; }
set -a
source "$root/settings.sh"
set +a
exec python3 "$root/scripts/portable_robot.py" setup
