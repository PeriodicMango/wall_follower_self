#!/usr/bin/env bash
# 把 ~/bags 下的产物（maze_geometry.txt、bag 目录等）通过 HTTP 暴露给同一局域网，
# 便于从 WSL / droplet 侧取回。
#
# 用法:
#   ./serve_artifacts.sh          # 默认 8000 端口
#   ./serve_artifacts.sh 8080     # 指定端口
# Ctrl-C 停止。只读，不会写入任何东西。
set -euo pipefail

PORT="${1:-8000}"
DIR="${HOME}/bags"
mkdir -p "$DIR"
cd "$DIR"

IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "服务目录: $DIR"
echo "本机:      http://localhost:${PORT}/"
[ -n "$IP" ] && echo "局域网:    http://${IP}:${PORT}/"
echo "取单个文件: curl -sS http://${IP:-<IP>}:${PORT}/maze_geometry.txt"
echo "Ctrl-C 停止"
echo

exec python3 -m http.server "$PORT" --bind 0.0.0.0
