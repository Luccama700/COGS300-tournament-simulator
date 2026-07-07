**DAgger with keep-last checkpointing on an unweighted growing aggregate collapses in late iterations — fix the loop, don't add iterations.**

Observed: rollout completions peaked at iteration 1 (9/50) then regressed;
by the last iteration the policy produced spin-event storms (hundreds of
watchdog trips). Adding iterations amplified the problem.

Diagnosed defects in the loop as shipped (`training/dagger.py`):

1. The aggregate dataset grows with recovery-heavy rows each iteration, so
   later training sees mostly flailing states — no controlled base:recovery
   ratio.
2. Keep-last: the final checkpoint is kept even when an earlier iteration
   was closed-loop better. The loop discards its own best iterate.
3. Rollouts run guards-off while deployment runs guards-on, so the state
   distribution DAgger corrects for is not the deployment distribution.

Any DAgger revival should address all three; each is small with a clear
reason (controlled data ratio; per-iteration closed-loop selection with
keep-best; guards-on rollouts).

Full story: `docs/PROJECT_GUIDE.md` §4.6.
