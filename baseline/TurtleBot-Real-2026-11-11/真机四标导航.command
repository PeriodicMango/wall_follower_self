#!/bin/bash
set -e
base="$(cd "$(dirname "$0")" && pwd)"
if [[ "${1:-}" == "--help" ]]; then
  echo '手动考核 [目录]：用 2D Pose Estimate 定位，再点一次 2D Goal Pose，按 CSV 顺序前往各站并返回原起点。考核导航 [真机记录目录]：在原建图起点静止后自动定位并倒计时起跑，无需点击 RViz。按该轮 maps/landmarks.csv 的行顺序逐站访问，回到建图起点。预览考核 [目录] 只打印路线、不开车。可用 --order-csv 自选顺序文件、--expect-count 4 要求四标齐全。旧的「已有地标」和「随机地标」模式仅访问地标。导航中按回车停车。'
  exit 0
fi
case "${1:-}" in
  手动考核) shift; set -- --recorded --exam --rviz-start "$@" ;;
  考核导航) shift; set -- --recorded --exam "$@" ;;
  预览考核) shift; set -- --recorded --exam --preview "$@" ;;
  已有地标) shift; set -- --recorded "$@" ;;
  随机地标) shift; set -- --recorded --shuffle "$@" ;;
esac
exec /bin/bash "$base/scripts/real_env.sh" python3 -Bu "$base/scripts/manual_navigation.py" "$@"
