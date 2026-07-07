**The chassis is a 12×22 cm rectangle, not a 22 cm disc — grid inflation above 8 cm seals passages the robot actually fits through.**

Collision is oriented-rectangle (OBB) via Liang–Barsky, half-extents
11×7 cm including margin. The track contains ~20 cm slits that the chassis
passes end-on. A planner inflation radius >8 cm treats those slits as walls
and either finds no route or finds a worse one.

Division of labor (keep):

- Inflation (8 cm) handles *body clearance* along corridors.
- The densify+push pass in `expert_policy._push_from_corners` (12.5 cm from
  wall endpoints, after densifying to 5 cm spacing) handles *corner
  clearance* — pushing only vertices misses mid-segment passes near a
  corner, which is why densification exists.

Also verified: the interior slit route (through the 20 cm elbows) is at the
edge of drivability — the expert completed only ~53% through it — so the
wall-hug cost (1.6× for cells >35 cm from walls) that steers the route
along walls is load-bearing, both for the expert and for giving the policy
ultrasound landmarks instead of blind open-floor transit.

Full story: `docs/PROJECT_GUIDE.md` §4.1 and §3.
