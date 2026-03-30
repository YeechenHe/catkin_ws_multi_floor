#!/bin/bash
# 模块 C v3 小样本验证——仅 medium 工况 (0.4, 0.4, 0.2)，baseline + relocc 各 3 次
# 日志写入与 pilot 相同目录，便于一起解析
# 用法: 在仓库根目录执行 ./scripts/amcl/run_v3_pilot_medium_only.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$REPO_DIR/multi_floor_nav/experiment_logs_module_c_v3_pilot"
DURATION=200
RUNS=3

echo "[v3-pilot-medium] LOG_DIR=$LOG_DIR duration=${DURATION}s runs=$RUNS"
mkdir -p "$LOG_DIR"
cd "$REPO_DIR"

total=$((2 * RUNS))
count=0

for method in baseline relocc; do
    for run in $(seq 1 $RUNS); do
        count=$((count + 1))
        echo "[v3-pilot-medium] === [$count/$total] $method pert=medium (0.4,0.4,0.2) run $run ==="
        "$SCRIPT_DIR/run_one_experiment.sh" "$method" 0.4 0.4 0.2 "$run" "$DURATION" "$LOG_DIR"
    done
done

echo "[v3-pilot-medium] All $total runs done."
echo "[v3-pilot-medium] Parse with: python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
