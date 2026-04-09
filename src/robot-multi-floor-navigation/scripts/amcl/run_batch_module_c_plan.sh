#!/bin/bash
# 模块 C 单独实验 — 按《moduleC_experiment_plan》改进方案批量跑 Baseline vs relocc（关模块 A）
#
# 正式主工况（默认，不含无扰动）：
#   small     : (0.1, 0.1, 0.05)     d_planar≈0.141
#   medium    : (0.25, 0.25, 0.15)   d_planar≈0.354
#   boundary  : (0.30, 0.30, 0.18)   d_planar≈0.424
# 可选：
#   --with-none           额外加入 none (0,0,0)，仅作对照/烟测连通性（无扰动易收敛，不宜作正式主结论）
#   --with-large-boundary 加入 large_boundary (0.32, 0.32, 0.2) — 非单独主工况，仅作补充/与 SP-AMCL 命名对齐
#
# 模块 C 参数见 multi_floor_nav/configs/reloc_confidence_criterion.yaml（λ=20, τ=0.85, K=3, delay=6s）
#
# 用法:
#   ./run_batch_module_c_plan.sh [runs] [log_dir] [--with-none] [--with-large-boundary]
#   runs    每组每法重复次数，默认 10（改进文档建议 10～15）
#   log_dir 默认: multi_floor_nav/experiment_logs_module_c_v3
#
# 解析（跑完后）:
#   python3 scripts/amcl/parse_experiment_logs.py <log_dir>

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

RUNS=10
LOG_DIR="$REPO_DIR/multi_floor_nav/experiment_logs_module_c_v3"
INCLUDE_NONE=0
INCLUDE_LARGE_BOUNDARY=0
for arg in "$@"; do
    case "$arg" in
        --with-none) INCLUDE_NONE=1 ;;
        --with-large-boundary) INCLUDE_LARGE_BOUNDARY=1 ;;
        *) ;;
    esac
done
# 非 flag 参数：第一个数字为 runs，第一个含 / 的路径为 log_dir
for arg in "$@"; do
    [ "$arg" = "--with-large-boundary" ] && continue
    [ "$arg" = "--with-none" ] && continue
    if [[ "$arg" =~ ^[0-9]+$ ]]; then RUNS="$arg"; continue; fi
    if [[ "$arg" == */* ]] || [[ "$arg" == ./* ]] || [[ "$arg" == ../* ]]; then LOG_DIR="$arg"; continue; fi
done

mkdir -p "$LOG_DIR"

# name ox oy ot duration_sec — 默认仅主工况三条；none / large_boundary 由 flag 追加
PERTURBATIONS=()
if [ "$INCLUDE_NONE" -eq 1 ]; then
    PERTURBATIONS+=("none     0      0      0      200")
fi
PERTURBATIONS+=(
    "small    0.1    0.1    0.05   200"
    "medium   0.25   0.25   0.15   200"
    "boundary 0.30   0.30   0.18   220"
)
if [ "$INCLUDE_LARGE_BOUNDARY" -eq 1 ]; then
    PERTURBATIONS+=("large_boundary 0.32 0.32 0.2 240")
fi

total_runs=$(( ${#PERTURBATIONS[@]} * 2 * RUNS ))
current=0
start_time=$(date +%s)

echo "============================================================"
echo "[batch-C-plan] 模块 C 改进方案批量实验（Baseline vs relocc，无模块 A）"
echo "[batch-C-plan] 配置: reloc_confidence_criterion.yaml (λ=20, τ=0.85, K=3, delay=6s)"
echo "[batch-C-plan] 工况: small+medium+boundary$([ "$INCLUDE_NONE" -eq 1 ] && echo '+none' || echo '')$([ "$INCLUDE_LARGE_BOUNDARY" -eq 1 ] && echo '+large_boundary' || echo '')"
echo "[batch-C-plan] 每组每法 ${RUNS} 次, 总次数 = $total_runs"
echo "[batch-C-plan] 日志: $LOG_DIR"
echo "[batch-C-plan] 可选: none=$([ "$INCLUDE_NONE" -eq 1 ] && echo 是 || echo 否)  large_boundary=$([ "$INCLUDE_LARGE_BOUNDARY" -eq 1 ] && echo 是 || echo 否)"
echo "[batch-C-plan] 开始: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

for pert_spec in "${PERTURBATIONS[@]}"; do
    read -r pname ox oy ot duration <<< "$pert_spec"

    echo ""
    echo "============================================================"
    echo "[batch-C-plan] 工况: $pname  offset=($ox,$oy,$ot)  duration=${duration}s"
    echo "============================================================"

    for run in $(seq 1 "$RUNS"); do
        for method in baseline relocc; do
            current=$(( current + 1 ))
            elapsed=$(( $(date +%s) - start_time ))
            if [ "$current" -gt 1 ]; then
                avg_per_run=$(( elapsed / (current - 1) ))
                remaining=$(( avg_per_run * (total_runs - current + 1) ))
                eta_min=$(( remaining / 60 ))
            else
                eta_min="--"
            fi

            echo ""
            echo "--------------------------------------------------------------"
            echo "[batch-C-plan] [$current/$total_runs] $method | $pname | run=$run | ETA≈${eta_min}min"
            echo "--------------------------------------------------------------"

            "$SCRIPT_DIR/run_one_experiment.sh" \
                "$method" "$ox" "$oy" "$ot" "$run" \
                "$duration" "$LOG_DIR" \
                0 0 0 0 \
                || true

            echo "[batch-C-plan] 完成；暂停 10s..."
            sleep 10
        done
    done

    echo ""
    echo "[batch-C-plan] 工况 $pname 小结（含两次 RELOC_PASS 的 run 数）:"
    for method in baseline relocc; do
        pass_count=0
        for run in $(seq 1 "$RUNS"); do
            logfile="$LOG_DIR/${method}_pert_${ox}_${oy}_${ot}_run${run}.log"
            if [ -f "$logfile" ]; then
                reloc_pass=$(grep -c "RELOC_PASS" "$logfile" 2>/dev/null || true)
                if [ "${reloc_pass:-0}" -ge 2 ]; then
                    pass_count=$((pass_count + 1))
                fi
            fi
        done
        echo "  $method $pname: reloc_ok>=2  →  $pass_count/$RUNS"
    done
done

total_elapsed=$(( $(date +%s) - start_time ))
echo ""
echo "============================================================"
echo "[batch-C-plan] 全部 $total_runs 次结束"
echo "[batch-C-plan] 总耗时: $(( total_elapsed / 60 )) 分 $(( total_elapsed % 60 )) 秒"
echo "[batch-C-plan] 解析:"
echo "  python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
echo "============================================================"
