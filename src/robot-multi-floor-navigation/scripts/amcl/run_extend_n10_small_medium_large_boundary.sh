#!/bin/bash
# 一键把论文主对比工况从 n=5 补齐到 n=10：
#   small / medium / large_boundary × Baseline / SP-AMCL × (run6~run10)
#
# 用法:
#   ./run_extend_n10_small_medium_large_boundary.sh [start_run] [end_run] [log_dir]
#  - start_run 默认 6
#  - end_run   默认 10
#  - log_dir   默认 multi_floor_nav/experiment_logs_sp_amcl
#
# 说明:
#  - 每次实验会自动清理残留 ROS/Gazebo（run_one_experiment.sh 内部已做）
#  - SP-AMCL sigma 参数与 run_batch_comparison.sh 保持一致：L0(0.15,0.1) / L1(0.20,0.12)

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

START_RUN="${1:-6}"
END_RUN="${2:-10}"
LOG_DIR="${3:-$REPO_DIR/multi_floor_nav/experiment_logs_sp_amcl}"

# 与既有批量实验保持一致
DURATION=200
SLEEP_BETWEEN_RUNS=10

mkdir -p "$LOG_DIR"

PERTURBATIONS=(
  "small         0.1   0.1   0.05"
  "medium        0.25  0.25  0.15"
  "large_boundary 0.32  0.32  0.20"
)

num_pert=${#PERTURBATIONS[@]}
num_runs=$(( END_RUN - START_RUN + 1 ))
if [ "$num_runs" -le 0 ]; then
  echo "[extend-n10] Invalid run range: start_run=$START_RUN end_run=$END_RUN"
  exit 1
fi

total_runs=$(( num_pert * 2 * num_runs ))
current=0
start_time=$(date +%s)

echo "============================================================"
echo "[extend-n10] small / medium / large_boundary 补齐到 n=10（跑 run${START_RUN}~run${END_RUN}）"
echo "[extend-n10] 每个工况 × (Baseline + SP-AMCL) × $num_runs 次，共 $total_runs 次"
echo "[extend-n10] 单次时长: ${DURATION}s, 间隔: ${SLEEP_BETWEEN_RUNS}s"
echo "[extend-n10] 日志目录: $LOG_DIR"
echo "[extend-n10] 开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "============================================================"

for pert_spec in "${PERTURBATIONS[@]}"; do
  read -r pname ox oy ot <<< "$pert_spec"

  echo ""
  echo "============================================================"
  echo "[extend-n10] 开始工况: $pname ($ox, $oy, $ot)"
  echo "============================================================"

  for run in $(seq "$START_RUN" "$END_RUN"); do
    for method in baseline sp_amcl; do
      # Avoid overwriting existing logs (resume-friendly).
      log_file="$LOG_DIR/${method}_pert_${ox}_${oy}_${ot}_run${run}.log"
      if [ -f "$log_file" ]; then
        echo ""
        echo "--------------------------------------------------------------"
        echo "[extend-n10] [SKIP] $method | $pname ($ox,$oy,$ot) | run=$run"
        echo "[extend-n10]        已存在日志: $log_file"
        echo "--------------------------------------------------------------"
        continue
      fi

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
      echo "[extend-n10] [$current/$total_runs] $method | $pname ($ox,$oy,$ot) | run=$run | ETA: ${eta_min}min"
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

      echo "[extend-n10] 完成 $method $pname run=$run, 暂停 ${SLEEP_BETWEEN_RUNS}s..."
      sleep "$SLEEP_BETWEEN_RUNS"
    done
  done
done

total_elapsed=$(( $(date +%s) - start_time ))
echo ""
echo "============================================================"
echo "[extend-n10] 全部 $total_runs 次实验完成"
echo "[extend-n10] 总耗时: $(( total_elapsed / 60 )) 分 $(( total_elapsed % 60 )) 秒"
echo "[extend-n10] 结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
echo "[extend-n10] 日志目录: $LOG_DIR"
echo "[extend-n10] 解析命令:"
echo "  python3 scripts/amcl/parse_experiment_logs.py $LOG_DIR"
echo "============================================================"

