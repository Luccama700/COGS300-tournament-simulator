# Experiment log

One line per run: date, change, command (abridged), seeds, numbers. Newest
at the bottom. Claims marked **[claim]** used ≥30 episodes on seeds 31000+;
everything else is iteration on seeds 5000+. `$TRACK`/`$ROBOT`/`$PHYS` are
the standard v03 / front-IR / slow-physics configs (see `rebuild.ps1`).

| Date | Change | Command | Seeds | Result |
|---|---|---|---|---|
| 2026-07-04 | Expert gate, baseline | `evaluate_policy.py --policy expert … --episodes 15 --randomization mild --max-time 240` | 5000+ | 15/15 success, ~118 s/run |
| 2026-07-04 | Expert on none / heavy | same, `--randomization none` / `heavy` | 5000+ | 15/15 none; 14–15/15 heavy |
| 2026-07-04 | BC baseline: 96×96 MLP, 26 feats, ~334k rows (200 mild + 100 heavy eps) | `rebuild.ps1` | 5000+ | val 90–92% balanced; closed loop **0/20**; median closest ~216 cm; tape section solved |
| 2026-07-04 | DAgger β 0.3→0.04, 3–4 iters × 50 eps, keep-last | `python -m training.dagger …` | 5000+ | rollout completions peak 9/50 (iter 1), late iterations regress (spin storms); checkpoints 0/15 |
| 2026-07-05 | Owner reproduced full pipeline on target Windows machine | `rebuild.ps1` | 5000+ | 194/200 + 85/100 demos reach goal; val 91.7% balanced; closed loop 0/20 (stuck 3 / timeout 10 / lost 7), median closest 216.27 cm — matches this repo's numbers |
