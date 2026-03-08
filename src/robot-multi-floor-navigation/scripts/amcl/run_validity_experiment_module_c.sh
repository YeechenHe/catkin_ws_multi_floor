#!/bin/bash
# 模块 C 有效性验证实验：medium 扰动 (0.3, 0.3, 0.15)，baseline vs relocc 各 4 次，证明改进优于 Baseline
# 用法: 在仓库根目录执行 ./scripts/amcl/run_validity_experiment_module_c.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$REPO_DIR/multi_floor_nav/experiment_logs_module_c_validity"
DURATION=200

echo "[validity] LOG_DIR=$LOG_DIR duration=${DURATION}s"
mkdir -p "$LOG_DIR"
cd "$REPO_DIR"

for run in 1 2 3 4; do
  echo "[validity] === Baseline run $run ==="
  "$SCRIPT_DIR/run_one_experiment.sh" baseline 0.3 0.3 0.15 "$run" "$DURATION" "$LOG_DIR"
done
for run in 1 2 3 4; do
  echo "[validity] === Relocc (模块 C) run $run ==="
  "$SCRIPT_DIR/run_one_experiment.sh" relocc 0.3 0.3 0.15 "$run" "$DURATION" "$LOG_DIR"
done

echo "[validity] Done. Parse with: python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
