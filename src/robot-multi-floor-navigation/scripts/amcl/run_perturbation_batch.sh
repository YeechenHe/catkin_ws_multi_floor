#!/bin/bash
# 批量初值扰动对比实验：Baseline vs SP-AMCL × 多档扰动 × 每档多轮
# 用法: ./run_perturbation_batch.sh [runs_per_cell] [log_dir]
#   runs_per_cell 默认 5；log_dir 默认 multi_floor_nav/experiment_logs
# 扰动档: 0, 小(0.2,0.2,0.1), 中(0.4,0.4,0.2), 大(0.6,0.3,0.25)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
RUNS="${1:-5}"
LOG_DIR="${2:-$REPO_DIR/multi_floor_nav/experiment_logs}"
DURATION=200

mkdir -p "$LOG_DIR"
echo "[batch] runs_per_cell=$RUNS log_dir=$LOG_DIR duration=${DURATION}s"

# 扰动档: name ox oy ot
PERTURBATIONS=(
  "0 0 0 0"
  "small 0.2 0.2 0.1"
  "medium 0.4 0.4 0.2"
  "large 0.6 0.3 0.25"
)

for pert_spec in "${PERTURBATIONS[@]}"; do
  read -r pname ox oy ot <<< "$pert_spec"
  for method in baseline spamcl; do
    for run in $(seq 1 "$RUNS"); do
      echo "========== $method pert=$pname run=$run =========="
      "$SCRIPT_DIR/run_one_experiment.sh" "$method" "$ox" "$oy" "$ot" "$run" "$DURATION" "$LOG_DIR" || true
      echo "Pausing 10s before next run..."
      sleep 10
    done
  done
done

echo "[batch] All runs done. Logs in $LOG_DIR"
echo "Parse with: python3 $SCRIPT_DIR/parse_experiment_logs.py $LOG_DIR"
