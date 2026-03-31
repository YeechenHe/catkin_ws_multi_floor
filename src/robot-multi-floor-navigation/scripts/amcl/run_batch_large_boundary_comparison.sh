#!/bin/bash
# 批量对比实验（论文主 large 工况）：large_boundary × Baseline / SP-AMCL × N 次
# large_boundary: (0.32, 0.32, 0.20), d_planar≈0.453 < e_max=0.5（Baseline 非必死）
#
# 用法: ./run_batch_large_boundary_comparison.sh [runs] [log_dir]
#   runs    : 每组重复次数，默认 5
#   log_dir : 日志目录，默认 experiment_logs_sp_amcl

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RUNS="${1:-5}"
LOG_DIR="${2:-$REPO_DIR/multi_floor_nav/experiment_logs_sp_amcl}"

# 与 run_batch_comparison.sh 保持一致
DURATION=200
OX=0.32
OY=0.32
OT=0.20

mkdir -p "$LOG_DIR"

total_runs=$(( 2 * RUNS ))
current=0
start_time=$(date +%s)

echo "============================================================"
echo "[batch-large-boundary] SP-AMCL vs Baseline 批量对比实验"
echo "[batch-large-boundary] 工况: large_boundary ($OX, $OY, $OT)"
echo "[batch-large-boundary] 每组 $RUNS 次, 共 $total_runs 次实验"
echo "[batch-large-boundary] 单次时长: ${DURATION}s"
echo "[batch-large-boundary] 日志目录: $LOG_DIR"
echo "[batch-large-boundary] 开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

for run in $(seq 1 "$RUNS"); do
  for method in baseline sp_amcl; do
    current=$(( current + 1 ))
    elapsed=$(( $(date +%s) - start_time ))
    if [ $current -gt 1 ]; then
      avg_per_run=$(( elapsed / (current - 1) ))
      remaining=$(( avg_per_run * (total_runs - current + 1) ))
      eta_min=$(( remaining / 60 ))
    else
      eta_min="--"
    fi

    echo ""
    echo "--------------------------------------------------------------"
    echo "[batch-large-boundary] [$current/$total_runs] $method | large_boundary ($OX,$OY,$OT) | run=$run | ETA: ${eta_min}min"
    echo "--------------------------------------------------------------"

    if [ "$method" = "sp_amcl" ]; then
      "$SCRIPT_DIR/run_one_experiment.sh" \
        "$method" "$OX" "$OY" "$OT" "$run" \
        "$DURATION" "$LOG_DIR" \
        0.15 0.1 0.20 0.12 \
        || true
    else
      "$SCRIPT_DIR/run_one_experiment.sh" \
        "$method" "$OX" "$OY" "$OT" "$run" \
        "$DURATION" "$LOG_DIR" \
        0 0 0 0 \
        || true
    fi

    echo "[batch-large-boundary] 完成 $method large_boundary run=$run, 暂停 10s..."
    sleep 10
  done
done

total_elapsed=$(( $(date +%s) - start_time ))
echo ""
echo "============================================================"
echo "[batch-large-boundary] 全部 $total_runs 次实验完成"
echo "[batch-large-boundary] 总耗时: $(( total_elapsed / 60 )) 分 $(( total_elapsed % 60 )) 秒"
echo "[batch-large-boundary] 结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "[batch-large-boundary] 日志目录: $LOG_DIR"
echo "[batch-large-boundary] 解析命令:"
echo "  python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
echo "============================================================"

