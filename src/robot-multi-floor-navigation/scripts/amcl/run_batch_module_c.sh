#!/bin/bash
# 批量对比实验：模块 C（relocc）vs Baseline × small / medium / large × N 次
# 仅开启模块 C（use_covariance_reloc=true），不启用模块 A/B
#
# 用法: ./run_batch_module_c.sh [runs] [log_dir]
#   runs    : 每组重复次数，默认 5
#   log_dir : 日志目录，默认 experiment_logs_module_c_v2
#
# 参数配置（reloc_confidence_criterion.yaml）：
#   lambda=20.0, tau=0.85, K=3
#
# 工况扰动定义（与 SP-AMCL 批量实验保持一致）：
#   small  : (0.1, 0.1, 0.05)
#   medium : (0.25, 0.25, 0.15)
#   large  : (0.4, 0.4, 0.2)

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RUNS="${1:-5}"
LOG_DIR="${2:-$REPO_DIR/multi_floor_nav/experiment_logs_module_c_v2}"

# large 工况给更多时间（SP-AMCL 批量实验经验：large 需 200s+）
DURATION_SMALL=200
DURATION_MEDIUM=200
DURATION_LARGE=240

mkdir -p "$LOG_DIR"

PERTURBATIONS=(
    "large  0.4   0.4   0.2   $DURATION_LARGE"
    "medium 0.25  0.25  0.15  $DURATION_MEDIUM"
    "small  0.1   0.1   0.05  $DURATION_SMALL"
)

total_runs=$(( ${#PERTURBATIONS[@]} * 2 * RUNS ))
current=0
start_time=$(date +%s)

echo "============================================================"
echo "[batch-C] 模块 C vs Baseline 批量对比实验"
echo "[batch-C] 模块 C 参数: lambda=20.0, tau=0.85, K=3"
echo "[batch-C] 工况: large / medium / small"
echo "[batch-C] 每组 $RUNS 次, 共 $total_runs 次实验"
echo "[batch-C] 日志目录: $LOG_DIR"
echo "[batch-C] 开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

for pert_spec in "${PERTURBATIONS[@]}"; do
    read -r pname ox oy ot duration <<< "$pert_spec"

    echo ""
    echo "============================================================"
    echo "[batch-C] 开始工况: $pname ($ox, $oy, $ot) duration=${duration}s"
    echo "============================================================"

    for run in $(seq 1 "$RUNS"); do
        for method in baseline relocc; do
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
            echo "[batch-C] [$current/$total_runs] $method | $pname ($ox,$oy,$ot) | run=$run | ETA: ${eta_min}min"
            echo "--------------------------------------------------------------"

            "$SCRIPT_DIR/run_one_experiment.sh" \
                "$method" "$ox" "$oy" "$ot" "$run" \
                "$duration" "$LOG_DIR" \
                0 0 0 0 \
                || true

            echo "[batch-C] 完成 $method $pname run=$run, 暂停 10s..."
            sleep 10
        done
    done

    echo ""
    echo "[batch-C] 工况 $pname 完成，快速查看 RELOC_PASS:"
    for method in baseline relocc; do
        pass_count=0
        for run in $(seq 1 "$RUNS"); do
            logfile="$LOG_DIR/${method}_pert_${ox}_${oy}_${ot}_run${run}.log"
            if [ -f "$logfile" ]; then
                reloc_pass=$(grep -c "RELOC_PASS" "$logfile" 2>/dev/null || true)
                if [ "$reloc_pass" -ge 2 ]; then
                    pass_count=$((pass_count + 1))
                fi
            fi
        done
        echo "  $method $pname: pass $pass_count/$RUNS"
    done
done

total_elapsed=$(( $(date +%s) - start_time ))
echo ""
echo "============================================================"
echo "[batch-C] 全部 $total_runs 次实验完成"
echo "[batch-C] 总耗时: $(( total_elapsed / 60 )) 分 $(( total_elapsed % 60 )) 秒"
echo "[batch-C] 结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "[batch-C] 解析命令:"
echo "  python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
echo "============================================================"
