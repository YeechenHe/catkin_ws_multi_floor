#!/bin/bash
# SP-AMCL 模块 A 对比实验批量脚本
# 对比：Baseline（单点 initialpose，sigma=0）vs SP-AMCL（区域高斯先验，sigma=0.15/0.15/0.1）
# 仅开模块 A（use_region_init），关闭模块 B（use_ap_amcl=false）和模块 C（use_covariance_reloc=false）
#
# 用法: ./run_sp_amcl_experiment.sh [runs_per_cell] [log_dir]
#   runs_per_cell : 每组重复次数，默认 10
#   log_dir       : 日志输出目录，默认 multi_floor_nav/experiment_logs_sp_amcl
#
# 工况 4 级（优先级：large > medium > small > none）：
#   none   (0,    0,    0)     无扰动：验证不退化（none 工况 SP-AMCL 也传 sigma=0）
#   small  (0.1,  0.1,  0.05)  小扰动（0.141m）：偏差<判据，双轨指标揭示假通过
#   medium (0.25, 0.25, 0.15)  中扰动（0.354m）：接近判据边界，Baseline 开始失败，核心对比
#   large  (0.4,  0.4,  0.2)   大扰动（0.566m）：偏差>判据，Baseline 几乎无法通过，最强证据
#
# 日志命名：<method>_pert_<ox>_<oy>_<ot>_run<N>.log
# 解析：python3 scripts/amcl/parse_experiment_logs.py <log_dir>

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RUNS="${1:-10}"
LOG_DIR="${2:-$REPO_DIR/multi_floor_nav/experiment_logs_sp_amcl}"
DURATION=200   # 单次实验时长（秒），与模块 C 实验保持一致

# SP-AMCL 模块 A sigma 参数（与 configs/spa_amcl_region_init.yaml 一致）
# 修复缺陷一：per-floor sigma，L0/L1 分别配置
SIGMA_X_L0_SPAMCL="0.15"
SIGMA_Y_L0_SPAMCL="0.15"
SIGMA_YAW_L0_SPAMCL="0.1"
SIGMA_X_L1_SPAMCL="0.20"
SIGMA_Y_L1_SPAMCL="0.20"
SIGMA_YAW_L1_SPAMCL="0.12"

# Baseline：传统 AMCL，initialpose 协方差为 0（单点）；刻意不施加最小 σ，避免对比对象被「改进」。
# 公平性说明见 docs/amcl/03_region_init_experiment_plan.md §2.2。
SIGMA_X_L0_BASELINE="0"
SIGMA_Y_L0_BASELINE="0"
SIGMA_YAW_L0_BASELINE="0"
SIGMA_X_L1_BASELINE="0"
SIGMA_Y_L1_BASELINE="0"
SIGMA_YAW_L1_BASELINE="0"

mkdir -p "$LOG_DIR"
echo "============================================================"
echo "[sp_amcl_exp] SP-AMCL Module A vs Baseline"
echo "[sp_amcl_exp] runs_per_cell=$RUNS  duration=${DURATION}s"
echo "[sp_amcl_exp] log_dir=$LOG_DIR"
echo "============================================================"

# 扰动工况：name ox oy ot（按优先级排列：large 最先跑）
PERTURBATIONS=(
    "large  0.4   0.4   0.2"
    "medium 0.25  0.25  0.15"
    "small  0.1   0.1   0.05"
    "0      0     0     0"
)

total_runs=$(( ${#PERTURBATIONS[@]} * 2 * RUNS ))  # 4工况 × 2方法 × RUNS
current=0

for pert_spec in "${PERTURBATIONS[@]}"; do
    read -r pname ox oy ot <<< "$pert_spec"

    for method in baseline spamcl; do
        # 修复缺陷一：按 method 选择 per-floor sigma（传给 run_one_experiment.sh）
        # 修复问题一：none 工况下 SP-AMCL 也使用 sigma=0（无扰动不需要散布，避免退化）
        if [ "$method" = "spamcl" ] && [ "$pname" != "0" ]; then
            SX_L0="$SIGMA_X_L0_SPAMCL";   SY_L0="$SIGMA_Y_L0_SPAMCL";   SYAW_L0="$SIGMA_YAW_L0_SPAMCL"
            SX_L1="$SIGMA_X_L1_SPAMCL";   SY_L1="$SIGMA_Y_L1_SPAMCL";   SYAW_L1="$SIGMA_YAW_L1_SPAMCL"
        else
            SX_L0="$SIGMA_X_L0_BASELINE"; SY_L0="$SIGMA_Y_L0_BASELINE"; SYAW_L0="$SIGMA_YAW_L0_BASELINE"
            SX_L1="$SIGMA_X_L1_BASELINE"; SY_L1="$SIGMA_Y_L1_BASELINE"; SYAW_L1="$SIGMA_YAW_L1_BASELINE"
        fi

        for run in $(seq 1 "$RUNS"); do
            current=$(( current + 1 ))
            echo ""
            echo "--------------------------------------------------------------"
            echo "[sp_amcl_exp] [$current/$total_runs] method=$method  pert=$pname($ox,$oy,$ot)  run=$run"
            echo "--------------------------------------------------------------"

            "$SCRIPT_DIR/run_one_experiment.sh" \
                "$method" "$ox" "$oy" "$ot" "$run" \
                "$DURATION" "$LOG_DIR" \
                "$SX_L0" "$SYAW_L0" \
                "$SX_L1" "$SYAW_L1" \
                || true   # 单次失败不终止整批

            echo "[sp_amcl_exp] Done run=$run. Pausing 10s..."
            sleep 10
        done
    done
done

echo ""
echo "============================================================"
echo "[sp_amcl_exp] All $total_runs runs finished."
echo "[sp_amcl_exp] Logs saved to: $LOG_DIR"
echo "[sp_amcl_exp] Parse results with:"
echo "  python3 $SCRIPT_DIR/parse_experiment_logs.py $LOG_DIR"
echo "  > $LOG_DIR/results_sp_amcl_full.txt"
echo "============================================================"
