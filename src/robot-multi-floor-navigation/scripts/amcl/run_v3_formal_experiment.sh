#!/bin/bash
# 正式对比实验 v3：3 扰动级别 x 2 方法 x 3 次 = 18 runs
# 扰动：none(0,0,0) / small(0.2,0.2,0.1) / medium(0.3,0.3,0.15)
# 方法：baseline / relocc（模块 C）
# 时长：360s（含导航到 L1 目标点的完整流程）
# 初始协方差：sigma_xy=0.15m, sigma_yaw=0.05rad（让 AMCL 粒子有初始展开）
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)/multi_floor_nav/experiment_logs_v3_formal"
DURATION=360
SIGMA_XY=0.15
SIGMA_YAW=0.05

echo "======================================================"
echo " 正式对比实验 v3  (18 runs, ${DURATION}s each)"
echo " Log dir: $LOG_DIR"
echo "======================================================"

run() {
    local method="$1" ox="$2" oy="$3" ot="$4" run_id="$5"
    echo ""
    echo ">>> [$run_id/18] method=$method pert=($ox,$oy,$ot)"
    bash "$SCRIPT_DIR/run_one_experiment.sh" \
        "$method" "$ox" "$oy" "$ot" "$run_id" "$DURATION" "$LOG_DIR" \
        "$SIGMA_XY" "$SIGMA_YAW"
}

# none 扰动（0,0,0）
run baseline 0 0 0 1
run baseline 0 0 0 2
run baseline 0 0 0 3
run relocc   0 0 0 1
run relocc   0 0 0 2
run relocc   0 0 0 3

# small 扰动（0.2,0.2,0.1）
run baseline 0.2 0.2 0.1 1
run baseline 0.2 0.2 0.1 2
run baseline 0.2 0.2 0.1 3
run relocc   0.2 0.2 0.1 1
run relocc   0.2 0.2 0.1 2
run relocc   0.2 0.2 0.1 3

# medium 扰动（0.3,0.3,0.15）
run baseline 0.3 0.3 0.15 1
run baseline 0.3 0.3 0.15 2
run baseline 0.3 0.3 0.15 3
run relocc   0.3 0.3 0.15 1
run relocc   0.3 0.3 0.15 2
run relocc   0.3 0.3 0.15 3

echo ""
echo "======================================================"
echo " 所有 18 runs 完成，开始解析结果..."
echo "======================================================"
python3 "$SCRIPT_DIR/parse_experiment_logs.py" "$LOG_DIR"
echo "======================================================"
echo " 实验完成！结果已保存至 $LOG_DIR/results.csv"
echo "======================================================"
