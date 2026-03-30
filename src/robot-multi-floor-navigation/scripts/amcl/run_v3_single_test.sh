#!/bin/bash
# Single-pair test: 1x baseline + 1x relocc at small perturbation (0.2, 0.2, 0.1)
# Validates that nomotion update fix makes Module C's confidence check meaningful
set -e
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOG_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)/multi_floor_nav/experiment_logs_single_test"

echo "=== Single-pair test: baseline vs relocc, small perturbation ==="
echo "Log dir: $LOG_DIR"

echo ""
echo ">>> [1/2] baseline pert=0.2,0.2,0.1 run=1"
bash "$SCRIPT_DIR/run_one_experiment.sh" baseline 0.2 0.2 0.1 1 200 "$LOG_DIR"

echo ""
echo ">>> [2/2] relocc pert=0.2,0.2,0.1 run=1"
bash "$SCRIPT_DIR/run_one_experiment.sh" relocc 0.2 0.2 0.1 1 200 "$LOG_DIR"

echo ""
echo "=== All runs finished. Parsing results... ==="
python3 "$SCRIPT_DIR/parse_experiment_logs.py" "$LOG_DIR"
echo "=== Done ==="
