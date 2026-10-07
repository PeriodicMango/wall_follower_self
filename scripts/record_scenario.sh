#!/usr/bin/env bash
# 一次跑完一个场景：录包 → 起控制器 → 定时 → 停车 → 收尾 → 打印 bag 概况
#
# 前置（不在本脚本范围内）：T1 已经起了仿真，机器人停在该场景位姿上，
#                          并且 policy_probe.py 已确认命中预期分支。
#
# 用法:
#   ./record_scenario.sh S1          # 默认 60 秒
#   ./record_scenario.sh S1 90
#
# 产物:
#   ~/bags/sim/<场景名>_run1/         bag
#   /tmp/wf_<场景名>_run1.log         控制器 stdout
#   /tmp/bagrec_<场景名>.log          录包器 stdout
set -uo pipefail

NAME="${1:-}"
DUR="${2:-60}"
if [ -z "$NAME" ]; then
  echo "用法: $0 <场景名> [秒数]        例: $0 S1 60" >&2
  exit 1
fi

BAG="$HOME/bags/sim/${NAME}_run1"
LOG="/tmp/wf_${NAME}_run1.log"
BAGLOG="/tmp/bagrec_${NAME}.log"

if [ -e "$BAG" ]; then
  echo "目标已存在: $BAG" >&2
  echo "先改名或删除:  mv $BAG ${BAG}_old   或   rm -rf $BAG" >&2
  exit 1
fi

if ! ros2 topic list 2>/dev/null | grep -qx '/scan'; then
  echo "/scan 不存在 —— 仿真还没起或还没就绪" >&2
  exit 1
fi

mkdir -p "$(dirname "$BAG")"

echo "[1/5] 开始录包 -> $BAG"
ros2 bag record -o "$BAG" /scan /odom /cmd_vel /tf /tf_static > "$BAGLOG" 2>&1 &
BAG_PID=$!
sleep 2

echo "[2/5] 起控制器（stdout -> $LOG）"
ros2 run wall_follower wall_follower > "$LOG" 2>&1 &
WF_PID=$!
sleep 1

echo "[3/5] 跑 ${DUR} 秒……"
sleep "$DUR"

echo "[4/5] 停车"
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.0}, angular: {z: 0.0}}' >/dev/null 2>&1 || true
sleep 1

echo "[5/5] 收尾"
kill -INT "$WF_PID" 2>/dev/null || true
sleep 2
kill -INT "$BAG_PID" 2>/dev/null || true      # SIGINT 让 rosbag2 把缓存写盘
sleep 3
kill -9 "$WF_PID" "$BAG_PID" 2>/dev/null || true

echo
echo "=== ros2 bag info ==="
ros2 bag info "$BAG" 2>&1 | sed -n '1,14p'
echo
echo "=== 控制器日志（预期只有 near_start 一行，或为空）==="
cat "$LOG" 2>/dev/null
echo
echo "下一步:"
echo "  python3 scripts/policy_replay.py $BAG | tee ~/bags/replay_${NAME}_run1.txt"
