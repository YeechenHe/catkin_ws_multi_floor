#!/bin/bash
# 从已有 partial 结果继续：补跑 small（缺 baseline 2–3 + relocc 1–3）+ 全部 medium
# 前置：experiment_logs_v3_formal 中已有 none×6 + baseline small run1
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)/multi_floor_nav/experiment_logs_v3_formal"
DURATION=360
SIGMA_XY=0.15
SIGMA_YAW=0.05

echo "======================================================"
echo " v3 formal experiment CONTINUE: 11 runs, ${DURATION}s each"
echo " Log dir: $LOG_DIR"
echo "======================================================"

run() {
    local method="$1" ox="$2" oy="$3" ot="$4" run_id="$5"
    echo ""
    echo ">>> method=$method pert=($ox,$oy,$ot) run=$run_id"
    bash "$SCRIPT_DIR/run_one_experiment.sh" \
        "$method" "$ox" "$oy" "$ot" "$run_id" "$DURATION" "$LOG_DIR" \
        "$SIGMA_XY" "$SIGMA_YAW"
}

# small 补跑
run baseline 0.2 0.2 0.1 2
run baseline 0.2 0.2 0.1 3
run relocc   0.2 0.2 0.1 1
run relocc   0.2 0.2 0.1 2
run relocc   0.2 0.2 0.1 3

# medium 全部
run baseline 0.3 0.3 0.15 1
run baseline 0.3 0.3 0.15 2
run baseline 0.3 0.3 0.15 3
run relocc   0.3 0.3 0.15 1
run relocc   0.3 0.3 0.15 2
run relocc   0.3 0.3 0.15 3

echo ""
echo "======================================================"
echo " Continue batch done. Parsing all logs..."
echo "======================================================"
python3 "$SCRIPT_DIR/parse_experiment_logs.py" "$LOG_DIR" | tee "$LOG_DIR/results_after_continue.txt"
echo "======================================================"
echo " Results: $LOG_DIR/results_after_continue.txt"
echo "======================================================"
