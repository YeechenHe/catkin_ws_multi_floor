#!/bin/bash
# 针对传统 AMCL 定位方案，按指定工况分别运行：
# 1) 原始 Improved A*（smoothing=false）
# 2) 平滑 Improved A*（smoothing=true）
#
# 规划算法对比实验不再使用 SP-AMCL，统一采用 baseline（传统 AMCL）
# 以减少定位侧干扰变量，并尽量对齐参考工程中更稳定的定位方案。
# 默认仅运行一个指定工况，便于在 Gazebo 可视化下人工观察和必要时调整状态。
# 若 SCENARIO=all，则按 none/small/medium/large 顺序全部执行。

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

DURATION="${1:-140}"
SCENARIO="${2:-none}"
RAW_LOG_DIR="${3:-$REPO_DIR/multi_floor_nav/experiment_logs_smoothing_raw}"
SMOOTH_LOG_DIR="${4:-$REPO_DIR/multi_floor_nav/experiment_logs_smoothing_smooth}"
USE_GUI="${5:-true}"
USE_HEADLESS="${6:-false}"

run_case() {
  local label="$1"
  local ox="$2"
  local oy="$3"
  local ot="$4"

  echo "============================================================"
  echo "[smooth_compare] RAW   label=$label pert=($ox,$oy,$ot) loc=baseline_amcl"
  echo "============================================================"
  "$SCRIPT_DIR/run_one_experiment.sh" \
    baseline "$ox" "$oy" "$ot" 1 \
    "$DURATION" "$RAW_LOG_DIR" \
    0.20 0.12 0.20 0.12 0 -1 false "$USE_GUI" "$USE_HEADLESS" false

  echo "============================================================"
  echo "[smooth_compare] SMOOTH label=$label pert=($ox,$oy,$ot) loc=baseline_amcl"
  echo "============================================================"
  "$SCRIPT_DIR/run_one_experiment.sh" \
    baseline "$ox" "$oy" "$ot" 1 \
    "$DURATION" "$SMOOTH_LOG_DIR" \
    0.20 0.12 0.20 0.12 0 -1 true "$USE_GUI" "$USE_HEADLESS" false
}

case "$SCENARIO" in
  none)
    run_case none 0 0 0
    ;;
  small)
    run_case small 0.1 0.1 0.05
    ;;
  medium)
    run_case medium 0.25 0.25 0.15
    ;;
  large)
    run_case large 0.4 0.4 0.2
    ;;
  all)
    run_case none 0 0 0
    run_case small 0.1 0.1 0.05
    run_case medium 0.25 0.25 0.15
    run_case large 0.4 0.4 0.2
    ;;
  *)
    echo "Unsupported scenario: $SCENARIO"
    echo "Usage: $0 [duration_sec] [none|small|medium|large|all] [raw_log_dir] [smooth_log_dir] [gui] [headless]"
    exit 1
    ;;
esac

echo "[smooth_compare] Done."
echo "  scenario:   $SCENARIO"
echo "  raw logs:    $RAW_LOG_DIR"
echo "  smooth logs: $SMOOTH_LOG_DIR"
