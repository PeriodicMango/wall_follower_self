#!/bin/bash
set -euo pipefail
base="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$base/runs"
bash "$base/scripts/real_env.sh" python3 -Bu "$base/scripts/vision/camera_preview.py" 2>&1 | tee -a "$base/runs/camera-preview.log"
