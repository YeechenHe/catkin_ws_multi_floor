#!/bin/bash
# 批量实验：模块A+C large 工况，优化参数 K=2 + per-floor timeout(L0=25s,L1=45s)
# n=5 次，替换旧的 K=3 数据

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="${1:-$REPO_DIR/multi_floor_nav/experiment_logs_sp_amcl}"

SIGMA_XY_L0=0.15; SIGMA_YAW_L0=0.1
SIGMA_XY_L1=0.20; SIGMA_YAW_L1=0.12
RELOC_K=2          # large 工况放宽至 K=2
DURATION=360       # large 工况实验时长

total=5
start_time=$(date +%s)

echo "============================================================"
echo "[batch-large-K2] 模块A+C large 工况批量实验（K=2，n=5）"
echo "[batch-large-K2] per-floor timeout: L0=25s, L1=45s（代码已固化）"
echo "[batch-large-K2] 日志目录: $LOG_DIR"
echo "[batch-large-K2] 开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

for run_id in 1 2 3 4 5; do
    elapsed=$(( $(date +%s) - start_time ))
    if [ $run_id -gt 1 ]; then
        avg=$(( elapsed / (run_id - 1) ))
        eta_min=$(( avg * (total - run_id + 1) / 60 ))
    else
        eta_min="--"
    fi

    echo ""
    echo "--------------------------------------------------------------"
    echo "[batch-large-K2] [$run_id/$total] sp_amcl_c | large (0.4,0.4,0.2) | K=$RELOC_K | dur=${DURATION}s | ETA: ${eta_min}min"
    echo "--------------------------------------------------------------"

    "$SCRIPT_DIR/run_one_experiment.sh" \
        sp_amcl_c 0.4 0.4 0.2 "$run_id" \
        "$DURATION" "$LOG_DIR" \
        "$SIGMA_XY_L0" "$SIGMA_YAW_L0" "$SIGMA_XY_L1" "$SIGMA_YAW_L1" \
        "$RELOC_K" \
        || true

    echo "[batch-large-K2] 完成 run=$run_id，暂停 10s..."
    sleep 10
done

total_elapsed=$(( $(date +%s) - start_time ))
echo ""
echo "============================================================"
echo "[batch-large-K2] 全部 $total 次实验完成"
echo "[batch-large-K2] 总耗时: $(( total_elapsed / 60 )) 分 $(( total_elapsed % 60 )) 秒"
echo "[batch-large-K2] 结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "[batch-large-K2] 解析命令:"
echo "  grep -h 'RELOC_PASS' $LOG_DIR/sp_amcl_c_pert_0.4_0.4_0.2_run*.log"
echo "============================================================"
