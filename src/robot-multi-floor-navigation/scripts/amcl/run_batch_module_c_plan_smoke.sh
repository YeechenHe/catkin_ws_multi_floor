#!/bin/bash
# 模块 C 改进方案 — 各工况「单次」烟测（每工况：Baseline run1 + relocc run1）
#
# 用途：全批量前验证 launch、日志、RELOC_PASS 是否正常。
# 默认与正式批量一致：主工况 3 条（small / medium / boundary）→ 共 6 次。
#   --with-none           再加 none → 8 次
#   --with-large-boundary 再加 large_boundary → +2 次（可与 --with-none 叠加）
#
# 用法:
#   ./run_batch_module_c_plan_smoke.sh [日志目录] [--with-none] [--with-large-boundary]
#   日志目录默认: multi_floor_nav/experiment_logs_module_c_v3_smoke
#
# 跑完后:
#   python3 scripts/amcl/parse_experiment_logs.py <日志目录>

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

LOG_DIR="$REPO_DIR/multi_floor_nav/experiment_logs_module_c_v3_smoke"
EXTRA=()
for arg in "$@"; do
    case "$arg" in
        --with-large-boundary|--with-none) EXTRA+=("$arg") ;;
        */*|./*|../*) LOG_DIR="$arg" ;;
    esac
done

echo "============================================================"
echo "[smoke] 单次烟测：每工况 baseline run1 + relocc run1（默认 3 主工况 → 6 次）"
echo "[smoke] 日志目录: $LOG_DIR"
echo "============================================================"

exec "$SCRIPT_DIR/run_batch_module_c_plan.sh" 1 "$LOG_DIR" "${EXTRA[@]}"
