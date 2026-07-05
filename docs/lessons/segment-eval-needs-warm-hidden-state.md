**Teleport and expert-prefix segment starts agree for our GRU (validated 2026-07-04) — and the scarier-looking teleport numbers were a metric artifact: `min_goal_dist` is misleading on a folded course. Judge progress by max route-arc reached, not euclidean goal distance.**

What happened, in order:

1. GRU v1 teleport segment eval read 0/6 everywhere with *later* spawns
   scoring worse (median closest 519 cm from maze_1of4 vs 217 cm from
   start). First interpretation: h=0 + mid-course odometer is off-manifold
   for a recurrent net.
2. Built the control: `--prefix` mode in `training/segment_eval.py` — the
   expert drives to the arc while the policy runtime observes every step
   (features, hidden state, observe_command), then control hands over. The
   hidden state at handover is training-identical by construction.
3. Prefix results matched teleport almost exactly (520 vs 519 cm at
   maze_1of4; 281 vs 281 at tape_end). The cold-start hypothesis was
   FALSE for this model — its effective memory horizon is short enough
   that h converges from the feature stream within a segment.

The real explanations:

- The policy simply dies fast in the maze wherever it starts (confirmed by
  two independent eval modes) — the maze is the weakness, full stop.
- "Later spawn = larger min_goal_dist" happens because the course is
  folded: the tape passes euclidean-close to the goal chamber, so dying on
  the tape reads ~215 cm while dying instantly at 53% arc reads ~520 cm.
  Corollary: every historical "median closest ~216 cm" number in this repo
  most likely means "died on the tape near the fold", NOT "got most of the
  way there".

Consequences (implemented):

- `evaluate_policy.py` tracks **max route-arc reached** (monotonic windowed
  projection, privileged, metric-only) and reports it per episode and as
  `median_max_arc`; `training/closed_loop.score_key` ranks checkpoints by
  (success, median_max_arc, −median_min_goal_dist). Success still dominates.
- Keep `--prefix` for future models with longer memory horizons: if prefix
  and teleport ever disagree, trust prefix.
