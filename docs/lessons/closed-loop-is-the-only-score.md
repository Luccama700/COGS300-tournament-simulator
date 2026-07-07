**Validation accuracy does not rank driving policies; only closed-loop evaluation does.**

The BC baseline scores 90–92% balanced per-frame validation accuracy and
0/20 closed loop. That is not a paradox — it is the compounding-error
signature of behavior cloning: tiny per-frame errors move the robot into
states the expert never visited, where the model has no supervision, and
errors snowball over the ~1,200-decision horizon.

Consequences, verified in this repo:

- Never select a checkpoint, tune a hyperparameter, or claim progress from
  validation accuracy. Use `evaluate_policy.py` success rate, then median
  closest-approach as the tiebreaker/progress signal.
- Iterate on seeds 5000+ with 10–15 episodes; claim only from ≥30 episodes
  on seeds 31000+ that never touched training or selection.
- Headline claims need a `--no-safeguards` run alongside, or guards may be
  doing the driving.

Full story: `docs/PROJECT_GUIDE.md` §4.5 (evaluation) and §6 (current state).
