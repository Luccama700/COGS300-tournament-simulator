**Corner pivots must fire on a sensor event (IR graze), not on pure odometry distance.**

Odometry scale error is randomized ±2% (matching real encoders). Over the
course, that is several centimeters of drift — larger than a corner trigger
window. A purely distance-triggered pivot therefore fires at the wrong
place relative to what the sensors show, which the policy experiences as
irreducible label noise: identical observations, different labels.

Current design (keep): *fire-armed* corners. The distance window (±12 cm)
only **arms** the corner; the actual pivot **fires** on the IR graze event,
which is observable. Angle-scaled pivot duration; each corner is one-shot
(consumed via a done-set); a fire cancels the first half of a committed
pivot.

Also verified: monotonic-progress pinning of the corner window deadlocks
(0/15) when a fire is missed — the one-shot done-set exists for that reason.

Full story: `docs/PROJECT_GUIDE.md` §4.1.
