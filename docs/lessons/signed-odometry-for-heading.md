**Heading estimates from direction-blind encoders are garbage after hard turns; the sim models the signed-odometry firmware fix — keep it that way.**

The real firmware counts encoder ticks without direction, so during an
in-place pivot both wheels "advance" and integrated heading barely changes —
while the robot has turned 90°. Features derived from that heading are
poison precisely at the moments that matter (turns).

`sim_env.py` therefore integrates **signed** wheel travel
(`d_theta = (right_signed − left_signed) · enc_scale / wheelbase`), which
corresponds to a documented two-line firmware change (sign ticks by
commanded motor direction). The README records this as a required hardware
change.

Consequences:

- Any new odometry-derived feature must assume signed travel and nothing
  more (no magic pose).
- The spin watchdog in `PolicyRuntime` uses heading only (250° net rotation)
  for the same reason — distance during a pivot is untrustworthy.
- If you ever evaluate a "no firmware change" scenario, heading features
  must be dropped, not degraded.

Full story: `docs/PROJECT_GUIDE.md` §5 (findings for the physical robot).
