**Never give the expert behavior whose trigger the policy cannot observe — it clones into noise.**

Three separate incidents, same root cause:

1. A *weaving* expert (added to make the tape line visible to front sensors
   more often) cloned into open-floor wandering. The weave phase was keyed
   to route position — hidden state the policy can't see — so the model
   learned "sometimes veer" with no way to know when to stop.
2. *Turn-release* (expert stops pivoting when heading aligns with the route)
   cloned into locked HARD_LEFT at p≈1.0 — endless spinning. The release
   trigger (true heading vs route tangent) was invisible; fixed by adding
   `turn_accum`/`heading_rate` features so release becomes observable.
3. *Distance-triggered corners* cloned into missed/false pivots (see
   `corner-triggers-need-sensor-events.md`).

Rule: before adding any expert behavior, name the observation channel the
policy would use to imitate it. If you can't, the behavior is training-data
poison, however well the expert drives with it.

Superseded designs (do not reintroduce): weaving; route-position-keyed
trajectory shaping of any kind. The current expert uses sensor-driven
bang-bang line following instead.

Full story: `docs/PROJECT_GUIDE.md` §4.1 (the observability incident table).
