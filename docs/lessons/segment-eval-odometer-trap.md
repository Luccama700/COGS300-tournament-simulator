**If an eval teleports the robot mid-route, it must also set the odometer to the matching arc length — otherwise the `distance` feature is out-of-distribution and the eval is invalid.**

Segment-start evaluation (spawning episodes at tape-end, maze mouth, spiral
exit, …) is the biggest iteration-speed win available, but it has one
correctness trap: the policy's feature vector includes accumulated odometer
distance. Training data at the maze mouth always shows ~X cm on the
odometer; a teleported episode that starts the odometer at 0 presents a
feature combination that never occurs in training, so failures there tell
you nothing about the policy.

Implementation note for whoever builds it: after `env.reset()`, set the
pose to the route point/tangent at the chosen arc length AND set the
environment's accumulated odometry distance to that arc length (± the usual
noise). Then per-segment success is meaningful and the weakest segment can
be targeted with extra training starts.

Status: built 2026-07-04 as `training/segment_eval.py` (SimEnv gained
`start_pose`/`odom_init`; odometer values calibrated per segment from expert
runs — measured *lower* than route arc because the robot rounds vertices).
Expert passes 4/4 from every teleport spawn, so the trap is handled. For
recurrent policies teleport spawns are still invalid for a second reason —
see segment-eval-needs-warm-hidden-state.md (`--prefix` mode).
