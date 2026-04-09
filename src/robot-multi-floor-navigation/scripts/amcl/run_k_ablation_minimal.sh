#!/bin/bash
# K 消融（最小次数）：固定 τ=0.85、λ=20（YAML），只比较 K∈{2,3}
#
# 设计：单工况 large_boundary (0.32,0.32,0.2)，共 3 次仿真
#   1) baseline run1
#   2) relocc K=3（override 0，用 YAML 默认）run2
#   3) relocc K=2（第 12 参覆盖）run3
# 同一日志目录内用不同 run_id 区分，避免 relocc 日志互相覆盖。
#
# 用法:
#   ./run_k_ablation_minimal.sh [日志目录]
#   默认: multi_floor_nav/experiment_logs_k_ablation_minimal
#
# 结束后:
#   python3 scripts/amcl/parse_experiment_logs.py <日志目录>

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="${1:-$REPO_DIR/multi_floor_nav/experiment_logs_k_ablation_minimal}"
mkdir -p "$LOG_DIR"

echo "============================================================"
echo "[K-ablation minimal] 工况: large_boundary (0.32,0.32,0.2) duration=240s"
echo "[K-ablation minimal] 共 3 次: baseline run1 | relocc K=3 run2 | relocc K=2 run3"
echo "[K-ablation minimal] 日志: $LOG_DIR"
echo "============================================================"

"$SCRIPT_DIR/run_one_experiment.sh" baseline 0.32 0.32 0.2 1 240 "$LOG_DIR"
echo "[K-ablation minimal] 暂停 10s..."
sleep 10

"$SCRIPT_DIR/run_one_experiment.sh" relocc 0.32 0.32 0.2 2 240 "$LOG_DIR" 0.15 0.1 0.15 0.1 0
echo "[K-ablation minimal] 暂停 10s..."
sleep 10

"$SCRIPT_DIR/run_one_experiment.sh" relocc 0.32 0.32 0.2 3 240 "$LOG_DIR" 0.15 0.1 0.15 0.1 2

echo ""
echo "============================================================"
echo "[K-ablation minimal] 全部结束。解析:"
echo "  python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
echo "============================================================"
