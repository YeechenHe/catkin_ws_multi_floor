#!/bin/bash
# 有效性实验跑完后执行：解析日志、保存 CSV、打印 Summary 供填 §6.6
# 用法: 在仓库根目录执行 ./scripts/amcl/run_after_validity_experiment.sh

set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
LOG_DIR="$REPO_DIR/multi_floor_nav/experiment_logs_module_c_validity"
CSV_PATH="$REPO_DIR/docs/amcl/reloc_criterion_validity_data.csv"

cd "$REPO_DIR"
n=$(find "$LOG_DIR" -maxdepth 1 -name "*.log" 2>/dev/null | wc -l)
echo "[after_validity] Found $n log(s) in $LOG_DIR (expected 8)."
if [ "$n" -lt 8 ]; then
  echo "[after_validity] Not all runs finished yet. Run again when 8 logs exist."
  exit 1
fi

echo "[after_validity] Parsing..."
python3 "$SCRIPT_DIR/parse_experiment_logs.py" "$LOG_DIR" > /tmp/validity_parse_full.txt 2>/dev/null || true
# CSV = 第 1 行表头 + 8 行数据
head -n 9 /tmp/validity_parse_full.txt > "$CSV_PATH"
echo "[after_validity] CSV saved to $CSV_PATH"
echo ""
echo "=== Summary（请据此填写 paper §6.6）==="
tail -n +11 /tmp/validity_parse_full.txt
