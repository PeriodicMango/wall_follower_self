#!/usr/bin/env bash
# 一次跑完一个场景：清残留 → 录包 → 起控制器 → 定时 → 停车 → 收尾 → 打印 bag 概况
#
# 前置（不在本脚本范围内）：T1 已经起了仿真，机器人停在该场景位姿上，
#                          并且 policy_probe.py 已确认命中预期分支。
#
# 用法:
#   ./record_scenario.sh S1            # 默认 60 秒，自动取下一个空闲的 _runN
#   ./record_scenario.sh S1 90         # 跑 90 秒
#   ./record_scenario.sh S1 60 2       # 强制用 _run2（已存在则拒绝）
#
# run 号规则: 从 1 开始找第一个不存在的 <场景名>_run<N>，所以同一个场景
#             连跑两次会依次落到 _run1、_run2，不需要手动改名字。
#
# 产物:
#   ~/bags/sim/<场景名>_run<N>/        bag
#   /tmp/wf_<场景名>_run<N>.log        控制器 stdout
#   /tmp/bagrec_<场景名>_run<N>.log    录包器 stdout
#
# 为什么要按进程名清残留: `kill -9` 杀的是 ros2 run 的 python 包装进程，
# 真正的节点是它的子进程，SIGKILL 不传递 —— 不清就会出现「幽灵控制器」
# 继续发 cmd_vel，下一次一起仿真机器人就自己动。
set -uo pipefail

NAME="${1:-}"
DUR="${2:-60}"
FORCE_RUN="${3:-}"

if [ -z "$NAME" ]; then
  echo "用法: $0 <场景名> [秒数] [run号]     例: $0 S1 60" >&2
  exit 1
fi

BASE="${NAME%_run*}"          # 允许误传 S1_run2，统一收回 S1
if [ -n "$FORCE_RUN" ]; then
  N="$FORCE_RUN"
else
  N=1
  while [ -e "$HOME/bags/sim/${BASE}_run${N}" ]; do
    N=$((N + 1))
  done
fi

BAG="$HOME/bags/sim/${BASE}_run${N}"
LOG="/tmp/wf_${BASE}_run${N}.log"
BAGLOG="/tmp/bagrec_${BASE}_run${N}.log"

CTRL='wall_follower/wall_follower'     # 真正的节点进程
WRAP='ros2 run wall_follower'          # ros2 run 包装进程

n_ctrl() { pgrep -f "$CTRL" 2>/dev/null | wc -l; }

kill_ctrl() {
  pkill -f "$WRAP" 2>/dev/null || true
  pkill -f "$CTRL" 2>/dev/null || true
  sleep 1
  pkill -9 -f "$WRAP" 2>/dev/null || true
  pkill -9 -f "$CTRL" 2>/dev/null || true
  sleep 1
}

# ---- 前置检查 ----------------------------------------------------------
if [ -e "$BAG" ]; then
  echo "目标已存在: $BAG" >&2
  echo "（只在显式指定 run 号时会出现）改名或删除后重试" >&2
  exit 1
fi

echo "场景 ${BASE}   使用 run${N}"
echo "  bag  -> $BAG"
echo "  日志 -> $LOG"

left=$(n_ctrl)
if [ "$left" -gt 0 ]; then
  echo "发现 $left 个残留的 wall_follower 节点 —— 先清掉"
  echo "（不清掉的话，新起的仿真一收到它们的 cmd_vel 就会自己动）"
  pgrep -af "$CTRL" | sed 's/^/    /'
  kill_ctrl
  echo "清理后剩 $(n_ctrl) 个"
fi

if ! ros2 topic list 2>/dev/null | grep -qx '/scan'; then
  echo "/scan 不存在 —— 仿真还没起或还没就绪" >&2
  exit 1
fi

mkdir -p "$(dirname "$BAG")"

# ---- 录包 → 控制器 → 定时 ----------------------------------------------
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
kill_ctrl                                     # 兜底：按进程名清掉子进程

left=$(n_ctrl)
if [ "$left" -gt 0 ]; then
  echo "警告: 仍有 $left 个控制器进程残留，再次清理" >&2
  kill_ctrl
  echo "剩余 $(n_ctrl) 个"
else
  echo "控制器已全部退出"
fi

echo
echo "=== ros2 bag info ==="
ros2 bag info "$BAG" 2>&1 | sed -n '1,14p'
echo
echo "=== 控制器日志（预期只有 near_start 一行，或为空）==="
cat "$LOG" 2>/dev/null
echo
echo "下一步:"
echo "  python3 scripts/policy_replay.py $BAG | tee ~/bags/replay_${BASE}_run${N}.txt"
