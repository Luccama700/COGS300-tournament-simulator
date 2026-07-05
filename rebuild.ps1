# rebuild.ps1 — regenerate training data, train, and evaluate the model.
# Windows PowerShell. Usage:   .\rebuild.ps1
# If script execution is blocked, run this first (safe, this window only):
#   Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass

$ErrorActionPreference = "Stop"

$TRACK = "configs/tracks/COGS_300_Tournament_Track/COGS_300_Tournament_Track_v03.yaml"
$ROBOT = "configs/robot-config-frontIR.yaml"
$PHYS  = "configs/physics-slow.yaml"

# --workers 4 uses all cores; if you hit a multiprocessing error on Windows,
# change both to --workers 1 (slower but bulletproof).
Write-Host "== 1/5  generating mild data (200 episodes) ==" -ForegroundColor Cyan
python generate_data_v2.py --track $TRACK --robot $ROBOT --physics $PHYS `
    --episodes 200 --randomization mild --output data/mild.csv --workers 4

Write-Host "== 2/5  generating heavy data (100 episodes) ==" -ForegroundColor Cyan
python generate_data_v2.py --track $TRACK --robot $ROBOT --physics $PHYS `
    --episodes 100 --randomization heavy --output data/heavy.csv --workers 4

Write-Host "== 3/5  merging ==" -ForegroundColor Cyan
python -m training.merge_datasets --out data/train.csv data/mild.csv data/heavy.csv

Write-Host "== 4/5  training (96x96 MLP) ==" -ForegroundColor Cyan
python -m training.train --data data/train.csv --out models/mine.npz --hidden 96 96

Write-Host "== 5/5  evaluating (20 episodes) ==" -ForegroundColor Cyan
python evaluate_policy.py --policy models/mine.npz `
    --track $TRACK --robot $ROBOT --physics $PHYS --episodes 20 --randomization mild --max-time 240

Write-Host ""
Write-Host "Done. Watch your model drive with:" -ForegroundColor Green
Write-Host "  python visualize.py --policy models/mine.npz --track $TRACK --robot $ROBOT --physics $PHYS"
