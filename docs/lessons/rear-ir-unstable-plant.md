**Rear-mounted IR line sensors make the plant itself unstable — the sim reproduces the real robot's spiral-hunting with zero ML involved. Stay on the front-IR config.**

With IR sensors behind the drive axle (the original build,
`configs/robot-config.yaml`), steering toward the hot sensor moves the
*sensors* off the line faster than it moves the robot onto it — an inverted
lever arm. The scripted expert (no learning anywhere) circles and hunts the
tape with that config: rescue-mode frames ~274/episode vs ~74 with front
IR. This reproduces the owner's real-robot spinning and is a *hardware*
finding, not a training defect.

Consequences:

- All training and evaluation use `configs/robot-config-frontIR.yaml`
  (sensors moved ahead of the axle; README documents the physical change).
- If someone asks to support the rear-IR build, that is a controls problem
  (needs damping/prediction), not more data — say so rather than training
  on an unstable plant.

Full story: `docs/PROJECT_GUIDE.md` §5.
