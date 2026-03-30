#!/bin/bash
# 批量补充实验：模块A+C（sp_amcl_c）补齐至 n=5
# 已有：small×1, medium×2, large×3
# 本次补：small run2~5, medium run3~5, large run4~5
# 共 9 次实验

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="${1:-$REPO_DIR/multi_floor_nav/experiment_logs_sp_amcl}"

# sigma 参数（与 run_batch_comparison.sh 保持一致）
SIGMA_XY_L0=0.15; SIGMA_YAW_L0=0.1
SIGMA_XY_L1=0.20; SIGMA_YAW_L1=0.12

# 实验列表：pert_name ox oy ot run_id duration
EXPERIMENTS=(
    "small  0.1  0.1  0.05  2  200"
    "small  0.1  0.1  0.05  3  200"
    "small  0.1  0.1  0.05  4  200"
    "small  0.1  0.1  0.05  5  200"
    "medium 0.25 0.25 0.15  3  200"
    "medium 0.25 0.25 0.15  4  200"
    "medium 0.25 0.25 0.15  5  200"
    "large  0.4  0.4  0.2   4  360"
    "large  0.4  0.4  0.2   5  360"
)

total=${#EXPERIMENTS[@]}
current=0
start_time=$(date +%s)

echo "============================================================"
echo "[batch-AC] 模块A+C 补充批量实验（凑齐 n=5）"
echo "[batch-AC] 共 $total 次实验"
echo "[batch-AC] 日志目录: $LOG_DIR"
echo "[batch-AC] 开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

for exp in "${EXPERIMENTS[@]}"; do
    read -r pname ox oy ot run_id duration <<< "$exp"
    current=$(( current + 1 ))

    elapsed=$(( $(date +%s) - start_time ))
    if [ $current -gt 1 ]; then
        avg=$(( elapsed / (current - 1) ))
        eta_min=$(( avg * (total - current + 1) / 60 ))
    else
        eta_min="--"
    fi

    echo ""
    echo "--------------------------------------------------------------"
    echo "[batch-AC] [$current/$total] sp_amcl_c | $pname ($ox,$oy,$ot) | run=$run_id | dur=${duration}s | ETA: ${eta_min}min"
    echo "--------------------------------------------------------------"

    "$SCRIPT_DIR/run_one_experiment.sh" \
        sp_amcl_c "$ox" "$oy" "$ot" "$run_id" \
        "$duration" "$LOG_DIR" \
        "$SIGMA_XY_L0" "$SIGMA_YAW_L0" "$SIGMA_XY_L1" "$SIGMA_YAW_L1" \
        || true

    echo "[batch-AC] 完成 sp_amcl_c $pname run=$run_id，暂停 10s..."
    sleep 10
done

total_elapsed=$(( $(date +%s) - start_time ))
echo ""
echo "============================================================"
echo "[batch-AC] 全部 $total 次实验完成"
echo "[batch-AC] 总耗时: $(( total_elapsed / 60 )) 分 $(( total_elapsed % 60 )) 秒"
echo "[batch-AC] 结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "[batch-AC] 解析命令:"
echo "  python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
echo "============================================================"
