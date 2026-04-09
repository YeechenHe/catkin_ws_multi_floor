#!/bin/bash
# large_boundary 下验证：放宽 L1 线误差门限能否恢复「两层闭环」
# 共 3 次：baseline（L0/L1 默认 0.5）| relocc L1 与 L0 共用 0.5（显式 -1）| relocc L1=0.65 对照
#
# 用法: ./run_max_linear_error_L1_verify.sh [日志目录]
# 解析: python3 scripts/amcl/parse_experiment_logs.py <日志目录>

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="${1:-$REPO_DIR/multi_floor_nav/experiment_logs_max_lin_L1_verify}"
mkdir -p "$LOG_DIR"

PERT="0.32 0.32 0.2"
SIG="0.15 0.1 0.15 0.1"
DUR=240

echo "============================================================"
echo "[max_lin_L1 verify] 工况: large_boundary ($PERT) duration=${DUR}s"
echo "[max_lin_L1 verify] 1) baseline run1  2) relocc L1=-1（共用 0.5）run2  3) relocc L1=0.65 run3"
echo "[max_lin_L1 verify] 日志: $LOG_DIR"
echo "============================================================"

"$SCRIPT_DIR/run_one_experiment.sh" baseline $PERT 1 "$DUR" "$LOG_DIR"
sleep 10

"$SCRIPT_DIR/run_one_experiment.sh" relocc $PERT 2 "$DUR" "$LOG_DIR" $SIG 0 -1
sleep 10

"$SCRIPT_DIR/run_one_experiment.sh" relocc $PERT 3 "$DUR" "$LOG_DIR" $SIG 0 0.65

echo ""
echo "============================================================"
echo "[max_lin_L1 verify] 结束。解析:"
echo "  python3 $SCRIPT_DIR/parse_experiment_logs.py $LOG_DIR"
echo "============================================================"
