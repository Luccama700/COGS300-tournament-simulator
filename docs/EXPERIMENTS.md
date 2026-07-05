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
| 2026-07-04 | Session-start gate: expert unchanged | `evaluate_policy.py --policy expert … --episodes 15 --randomization mild --max-time 240 --seed 5000` | 5000–5014 | 15/15, avg 112.5 s, 0 spins |
| 2026-07-04 | Session-start baseline: BC reference model | `evaluate_policy.py --policy models/policy_bc.npz … --episodes 10 --seed 5000` | 5000–5009 | 0/10 (8 lost / 2 timeout), median closest 309 cm, 40 spin events |
| 2026-07-04 | Segment-eval tool built (`training/segment_eval.py`, SimEnv start_pose+odom_init); expert gate from all 5 segment starts | `python -m training.segment_eval --policy expert --episodes 4` | 21000+ | 4/4 at every segment (start, tape_end, maze 1/4, 2/4, 3/4); odometry-heading tracking median err 4.1° — spawn+odom init valid. Measured: odometer reads *less* than route arc (721 vs 845 at tape end — robot rounds vertices); calibration is per-segment medians from expert runs |
| 2026-07-04 | GRU v1: 96-hidden GRU (36k params), torch BPTT-64 stateful, class-weighted, closed-loop checkpoint selection every 4 epochs (10 eps, seeds 20000+); numpy inference parity 3e-08 | `python -m training.train_gru --data data/train.csv --out models/policy_gru_v1.npz --epochs 40` | sel 20000–20009 | best ckpt epoch 24: **0/10**, median closest 211 cm, 150 spin evts. Val-balanced climbed 0.876→0.936 while closed-loop stayed 0 — recurrence alone does not beat covariate shift. Selection picked epoch 24 over val-best epoch 40 |
| 2026-07-04 | GRU v1 failure localization (project end pose onto route) | `python -m training.segment_eval --policy models/policy_gru_v1.npz --localize --episodes 8` | 5000–5007 | Deep but scattered deaths: 15%, 40%, 54%, 60%, 94%, 94%, 100%, 100% of course arc; one ep wedged 33 cm from goal. Course total 2232 cm, tape ends 845 |
| 2026-07-04 | GRU v1 teleport segment eval — INVALID for recurrent policies (kept as negative result) | `python -m training.segment_eval --policy models/policy_gru_v1.npz --episodes 6` | 21000+ | 0/6 every segment, all stuck; later spawns *worse* than course start (519 cm vs 217). Cause: h₀=0 + maze odometer is off-manifold for a GRU (lesson: segment-eval-needs-warm-hidden-state) |
