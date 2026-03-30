#!/bin/bash
# 模块 C v3 小样本验证：small + medium，每组 3 次，共 12 次
# 用法: 在仓库根目录执行 ./scripts/amcl/run_v3_pilot_experiment_module_c.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$REPO_DIR/multi_floor_nav/experiment_logs_module_c_v3_pilot"
DURATION=200
RUNS=3

echo "[v3-pilot] LOG_DIR=$LOG_DIR duration=${DURATION}s runs_per_group=$RUNS"
mkdir -p "$LOG_DIR"
cd "$REPO_DIR"

PERTS=("0.2 0.2 0.1" "0.4 0.4 0.2")
PERT_NAMES=("small" "medium")
METHODS=("baseline" "relocc")

total=$((${#PERTS[@]} * ${#METHODS[@]} * RUNS))
count=0

for pi in "${!PERTS[@]}"; do
    read -r dx dy dt <<< "${PERTS[$pi]}"
    pname="${PERT_NAMES[$pi]}"
    for method in "${METHODS[@]}"; do
        for run in $(seq 1 $RUNS); do
            count=$((count + 1))
            echo "[v3-pilot] === [$count/$total] $method pert=$pname ($dx,$dy,$dt) run $run ==="
            "$SCRIPT_DIR/run_one_experiment.sh" "$method" "$dx" "$dy" "$dt" "$run" "$DURATION" "$LOG_DIR"
        done
    done
done

echo "[v3-pilot] All $total runs done."
echo "[v3-pilot] Parse with: python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
