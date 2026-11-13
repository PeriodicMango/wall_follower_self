#!/bin/bash
# Build the native 2.3 controller without starting ROS nodes.
set -e
base="$(cd "$(dirname "$0")" && pwd)"
course="${TB3_COURSE:-$HOME/COMP3431/mac-native-gazebo-11.10.2}"
cd "$course"
export PIXI_CACHE_DIR="$course/.cache/pixi"
exec .tools/pixi run --locked bash -c '
  set -e
  source tools/env.sh
  variant="$1/local_variants/wall_follower_2_3"
  # The course linker cannot parse the macOS 27 arm64e.x1 TAPI stubs.
  course_sdk=/Library/Developer/CommandLineTools/SDKs/MacOSX26.5.sdk
  if [[ -n "${TB3_MACOS_SDK:-}" ]]; then export SDKROOT="$TB3_MACOS_SDK";
  elif [[ -d "$course_sdk" ]]; then export SDKROOT="$course_sdk";
  else export SDKROOT="$(xcrun --sdk macosx --show-sdk-path)"; fi
  export MACOSX_DEPLOYMENT_TARGET=15.0
  if [[ ! -d "$SDKROOT" ]]; then
    echo "未找到 macOS SDK。请检查 Command Line Tools 的安装目录。" >&2
    exit 1
  fi
  echo "正在编译本地雷达循墙 2.3……"
  cmake --fresh -S "$variant" -B "$variant/build" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_ARCHITECTURES=arm64 \
    -DCMAKE_OSX_SYSROOT="$SDKROOT" -DCMAKE_OSX_DEPLOYMENT_TARGET="$MACOSX_DEPLOYMENT_TARGET" \
    -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON -DCMAKE_INSTALL_RPATH="$PWD/.pixi/envs/default/lib" \
    -DCMAKE_C_COMPILER=/usr/bin/clang -DCMAKE_CXX_COMPILER=/usr/bin/clang++ \
    -DPython3_EXECUTABLE="$(command -v python3)"
  cmake --build "$variant/build"
  echo "编译完成，已生成 $variant/build/wall_follower_2_3"
' bash "$base"
