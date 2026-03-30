#!/bin/bash
# Extend medium perturbation experiments to n=8 per method.
# Existing runs: baseline 1-4, relocc 1-4
# This script adds runs 5-8 for each method.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)/multi_floor_nav/experiment_logs_micro_verify"
DURATION=260
SIGMA_XY=0.15
SIGMA_YAW=0.05

run() {
  local method="$1" run_id="$2"
  echo ""
  echo ">>> method=$method pert=(0.3,0.3,0.15) run=$run_id"
  bash "$SCRIPT_DIR/run_one_experiment.sh" \
    "$method" 0.3 0.3 0.15 "$run_id" "$DURATION" "$LOG_DIR" \
    "$SIGMA_XY" "$SIGMA_YAW"
}

echo "=== Medium extension to n=8 per method ==="
echo "log_dir=$LOG_DIR"

# baseline 5-8
run baseline 5
run baseline 6
run baseline 7
run baseline 8

# relocc 5-8
run relocc 5
run relocc 6
run relocc 7
run relocc 8

echo ""
echo "=== Parsing combined medium results ==="
python3 "$SCRIPT_DIR/parse_experiment_logs.py" "$LOG_DIR" | tee "$LOG_DIR/results_medium_n8.txt"
echo "=== Done: $LOG_DIR/results_medium_n8.txt ==="
