#!/bin/bash
set -e
base="$(cd "$(dirname "$0")" && pwd)"
usage() {
  echo '真机巡墙建图.command          启动 .103：wall_follower 2.3、See Marker（后拉 0.00 米）、相机、Cartographer 和 RViz'
  echo '真机巡墙建图.command 参数     在文本编辑器打开本机配置，不启动车轮'
  echo '真机巡墙建图.command 编译     重新编译本机 C++ 程序，不启动车轮'
  echo '巡墙中按回车停车保存，再按回车关闭本次建图窗口。'
}
if [[ $# -gt 1 ]]; then usage; exit 2; fi
case "${1:-}" in
  参数|--edit) exec /usr/bin/open -a TextEdit "$base/local_variants/wall_follower_2_3/wall_follower_2_3.yaml" ;;
  编译|--build) exec /bin/bash "$base/重新编译巡墙.command" ;;
  帮助|--help|-h) usage; exit 0 ;;
  "") ;;
  *) usage; exit 2 ;;
esac
exec /bin/bash "$base/scripts/real_env.sh" python3 -Bu "$base/scripts/manual_mapping.py"
