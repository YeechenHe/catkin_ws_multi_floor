#!/bin/bash
# 批量对比实验：small / medium / large × Baseline / SP-AMCL × N 次
# 用法: ./run_batch_comparison.sh [runs] [log_dir]
#   runs    : 每组重复次数，默认 5
#   log_dir : 日志目录，默认 experiment_logs_sp_amcl
#
# 实验顺序按工况优先级：large → medium → small
# 每个工况内先跑 Baseline 再跑 SP-AMCL，交替进行以减少系统性偏差

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RUNS="${1:-5}"
LOG_DIR="${2:-$REPO_DIR/multi_floor_nav/experiment_logs_sp_amcl}"
DURATION=200

mkdir -p "$LOG_DIR"

PERTURBATIONS=(
    "large  0.4   0.4   0.2"
    "medium 0.25  0.25  0.15"
    "small  0.1   0.1   0.05"
)

total_runs=$(( ${#PERTURBATIONS[@]} * 2 * RUNS ))
current=0
start_time=$(date +%s)

echo "============================================================"
echo "[batch] SP-AMCL vs Baseline 批量对比实验"
echo "[batch] 工况: large / medium / small"
echo "[batch] 每组 $RUNS 次, 共 $total_runs 次实验"
echo "[batch] 单次时长: ${DURATION}s"
echo "[batch] 预计总时长: $(( total_runs * (DURATION + 80) / 60 )) 分钟"
echo "[batch] 日志目录: $LOG_DIR"
echo "[batch] 开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

for pert_spec in "${PERTURBATIONS[@]}"; do
    read -r pname ox oy ot <<< "$pert_spec"

    echo ""
    echo "============================================================"
    echo "[batch] 开始工况: $pname ($ox, $oy, $ot)"
    echo "============================================================"

    for run in $(seq 1 "$RUNS"); do
        for method in baseline sp_amcl; do
            current=$(( current + 1 ))
            elapsed=$(( $(date +%s) - start_time ))
            if [ $current -gt 1 ]; then
                avg_per_run=$(( elapsed / (current - 1) ))
                remaining=$(( avg_per_run * (total_runs - current + 1) ))
                eta_min=$(( remaining / 60 ))
            else
                eta_min="--"
            fi

            echo ""
            echo "--------------------------------------------------------------"
            echo "[batch] [$current/$total_runs] $method | $pname ($ox,$oy,$ot) | run=$run | ETA: ${eta_min}min"
            echo "--------------------------------------------------------------"

            if [ "$method" = "sp_amcl" ]; then
                "$SCRIPT_DIR/run_one_experiment.sh" \
                    "$method" "$ox" "$oy" "$ot" "$run" \
                    "$DURATION" "$LOG_DIR" \
                    0.15 0.1 0.20 0.12 \
                    || true
            else
                "$SCRIPT_DIR/run_one_experiment.sh" \
                    "$method" "$ox" "$oy" "$ot" "$run" \
                    "$DURATION" "$LOG_DIR" \
                    0 0 0 0 \
                    || true
            fi

            echo "[batch] 完成 $method $pname run=$run, 暂停 10s..."
            sleep 10
        done
    done

    echo ""
    echo "[batch] 工况 $pname 完成，快速查看 L1 RELOC_PASS:"
    for method in baseline sp_amcl; do
        pass_count=0
        for run in $(seq 1 "$RUNS"); do
            logfile="$LOG_DIR/${method}_pert_${ox}_${oy}_${ot}_run${run}.log"
            if [ -f "$logfile" ]; then
                l1_pass=$(grep -c "level=1.*RELOC_PASS\|RELOC_PASS.*level=1" "$logfile" 2>/dev/null || \
                          grep "RELOC_PASS" "$logfile" 2>/dev/null | wc -l)
                if [ "$l1_pass" -ge 2 ]; then
                    pass_count=$((pass_count + 1))
                fi
            fi
        done
        echo "  $method $pname: L1 pass $pass_count/$RUNS"
    done
done

total_elapsed=$(( $(date +%s) - start_time ))
echo ""
echo "============================================================"
echo "[batch] 全部 $total_runs 次实验完成"
echo "[batch] 总耗时: $(( total_elapsed / 60 )) 分 $(( total_elapsed % 60 )) 秒"
echo "[batch] 结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "[batch] 日志目录: $LOG_DIR"
echo "============================================================"
