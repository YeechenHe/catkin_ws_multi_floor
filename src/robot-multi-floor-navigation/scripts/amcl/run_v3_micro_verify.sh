#!/bin/bash
# Micro verify: only 2 runs (baseline vs relocc) on medium perturbation.
# Goal: quickly verify whether Module C parameters produce meaningful separation.
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)/multi_floor_nav/experiment_logs_micro_verify"
DURATION=260
SIGMA_XY=0.15
SIGMA_YAW=0.05

echo "=== Micro verify: baseline vs relocc, medium perturbation (0.3,0.3,0.15) ==="
echo "Log dir: $LOG_DIR"

echo ""
echo ">>> [1/2] baseline pert=0.3,0.3,0.15 run=1"
bash "$SCRIPT_DIR/run_one_experiment.sh" baseline 0.3 0.3 0.15 1 "$DURATION" "$LOG_DIR" "$SIGMA_XY" "$SIGMA_YAW"

echo ""
echo ">>> [2/2] relocc pert=0.3,0.3,0.15 run=1"
bash "$SCRIPT_DIR/run_one_experiment.sh" relocc 0.3 0.3 0.15 1 "$DURATION" "$LOG_DIR" "$SIGMA_XY" "$SIGMA_YAW"

echo ""
echo "=== Parsing micro results ==="
python3 "$SCRIPT_DIR/parse_experiment_logs.py" "$LOG_DIR" | tee "$LOG_DIR/results_micro.txt"
echo "=== Done ==="
