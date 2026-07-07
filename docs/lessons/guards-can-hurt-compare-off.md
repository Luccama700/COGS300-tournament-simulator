**If a policy scores worse WITH safety guards than without, suspect guard false positives before blaming the policy — and calibrate guard thresholds against the expert's legitimate behavior envelope.**

Measured (2026-07-04): GRU v1 guards-on median max-arc 655 cm with 4/10
episodes lost; guards-off 735 cm with 0 lost. Cause: the spin watchdog
(250° net rotation per 3 s window) false-tripped ~21×/episode on the tape
zigzag, where two same-direction ~135° corners legitimately occur inside
one window (~270°). Each trip forced a 6-step FORWARD burst mid-corner,
throwing the robot off the line — the "protection" was creating the lost
episodes.

The fix was calibration, not removal: a genuinely locked hard turn rotates
~564°/window (188°/s), so threshold 340° separates real spins from legal
corner pairs with margin both ways. Guards-on then beat guards-off (median
arc 878 vs 735 cm) — protection with no false-positive tax.

Rules distilled:

- Run the guards-on/guards-off comparison on ARC PROGRESS (not just
  success) for every model generation; a guard tax shows up there first.
- Before setting any guard threshold, compute the maximum value the
  EXPERT's legitimate behavior produces (here: two-corner window rotation)
  and the minimum value the failure mode produces (locked-turn rotation);
  the threshold belongs between them, not at a round number.
- Guard changes are loud: log them in docs/EXPERIMENTS.md with the
  before/after numbers (done for this one).
