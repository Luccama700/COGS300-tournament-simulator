# rebuild.ps1 — regenerate maze training data, train the GRU, and evaluate.
# Windows PowerShell. Usage:   .\rebuild.ps1
# If script execution is blocked, run this first (safe, this window only):
#   Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
#
# Scope note (2026-07-06): the graded objective is the MAZE-ONLY track
# (see docs/AGENT_HANDOFF.md). Checkpoints are selected by closed-loop
# driving, never validation accuracy (docs/lessons/).

$ErrorActionPreference = "Stop"

$TRACK = "configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03_maze.yaml"
$ROBOT = "configs/robot-config-frontIR.yaml"
$PHYS  = "configs/physics-slow.yaml"

Write-Host "== 1/5  expert gate (must be ~15/15) ==" -ForegroundColor Cyan
python evaluate_policy.py --policy expert --track $TRACK --robot $ROBOT --physics $PHYS `
    --episodes 15 --randomization mild --max-time 150

Write-Host "== 2/5  generating mild data (200 episodes) ==" -ForegroundColor Cyan
python generate_data_v2.py --track $TRACK --robot $ROBOT --physics $PHYS `
    --episodes 200 --randomization mild --output data/maze2_mild.csv --workers 4

Write-Host "== 3/5  generating heavy data (100 episodes) ==" -ForegroundColor Cyan
python generate_data_v2.py --track $TRACK --robot $ROBOT --physics $PHYS `
    --episodes 100 --randomization heavy --output data/maze2_heavy.csv --workers 4

Write-Host "== 4/5  merge + train GRU (closed-loop checkpoint selection) ==" -ForegroundColor Cyan
python -m training.merge_datasets --out data/maze2_train.csv data/maze2_mild.csv data/maze2_heavy.csv
python -m training.train_gru --data data/maze2_train.csv --out models/policy_gru_maze.npz `
    --epochs 40 --track $TRACK

Write-Host "== 5/5  evaluating (20 episodes) ==" -ForegroundColor Cyan
python evaluate_policy.py --policy models/policy_gru_maze.npz `
    --track $TRACK --robot $ROBOT --physics $PHYS --episodes 20 --randomization mild --max-time 150

Write-Host ""
Write-Host "Done. Watch your model drive with:" -ForegroundColor Green
Write-Host "  python visualize.py --policy models/policy_gru_maze.npz --track $TRACK --robot $ROBOT --physics $PHYS"
