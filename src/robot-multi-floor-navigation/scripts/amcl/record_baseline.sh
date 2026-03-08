#!/bin/bash
# 录制 Baseline-AMCL 实验数据，用于后续与 SPA-AMCL 对比
# 用法：在 roslaunch multi_floor_nav nav.launch 已启动的前提下，另开终端执行本脚本；
#       建议在发送 /start 之前或同时启动本脚本，任务结束后 Ctrl+C 停止。
# 归类：AMCL 相关脚本统一放在 scripts/amcl/，参见 scripts/amcl/README.md

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BAG_DIR="${REPO_DIR}/multi_floor_nav/bagfiles"
BAG_NAME="baseline_$(date +%Y%m%d_%H%M%S).bag"

mkdir -p "$BAG_DIR"
cd "$BAG_DIR"

echo "[record_baseline] 录制话题到: $BAG_DIR/$BAG_NAME"
echo "[record_baseline] 话题: /amcl_pose /tf /odometry/filtered /move_base/status"
echo "[record_baseline] 按 Ctrl+C 结束录制"
rosbag record -O "$BAG_NAME" \
  /amcl_pose \
  /tf \
  /odometry/filtered \
  /move_base/status

echo "[record_baseline] 已保存: $BAG_DIR/$BAG_NAME"
