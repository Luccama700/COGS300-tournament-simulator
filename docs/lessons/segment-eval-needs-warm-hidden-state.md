**Teleport segment-starts are only valid for feedforward policies — a recurrent policy spawned mid-route with h=0 is off-manifold and fails in ways that say nothing about its real weakness.**

Measured (2026-07-04): GRU v1 from teleport spawns scored 0/6 at every
segment with all-stuck outcomes, and *later* spawns scored worse than the
course start (median closest 519 cm from maze_1of4 vs 217 cm from start) —
while full-course runs of the same model routinely passed those arcs. In
training the hidden state at arc s always carries the whole history since
episode start; h=0 paired with a mid-course odometer value is a feature
combination that never occurs, so the eval measures the mismatch, not the
policy.

The odometer-init fix (docs/lessons/segment-eval-odometer-trap.md) is
necessary but NOT sufficient once the policy has internal memory.

Fix for recurrent policies: **expert-prefix evaluation** — the expert drives
from the course start to the target arc while the policy's runtime observes
every step (features built, hidden state advanced, `observe_command` with
the expert's executed command), then control hands over. The hidden state at
handover is then history-consistent, and per-segment numbers mean what they
claim. Implemented as `--prefix` in `training/segment_eval.py`.

Teleport mode remains fine for MLPs (verified: expert 4/4 from every
teleport spawn — the plant and odometry init are sound).
