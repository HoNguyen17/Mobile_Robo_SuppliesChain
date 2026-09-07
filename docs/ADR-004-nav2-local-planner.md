# ADR-004: Nav2 Local Planner — Benchmark-Selected Between DWB and TEB

## Context
M3 (dynamic obstacle avoidance) depends heavily on local-planner behaviour. Nav2 ships DWB as the common default local planner, with TEB (Timed-Elastic-Band) available as an alternative that can produce smoother, more time-aware trajectories around moving obstacles at the cost of more tuning surface area. The team has limited time to tune either.

## Decision
The global planner (any stock Nav2 option, e.g. NavFn or a Smac variant per Phase 0 tutorial defaults) serves M2. For the local planner, DWB and TEB are both configured against scenario S-04 (dynamic NPC, `docs/test-plan.md`) during Phase 3, and the local planner is selected by a benchmark criterion: **whichever configuration achieves the lower collision count over N=20 runs in S-04 is adopted**; if collision counts tie, the one with the higher goal success rate over the same N=20 runs is adopted. This benchmark is time-boxed to 2 days; if inconclusive within that window, DWB is adopted by default because it ships with the tutorial's default configuration and requires less new tuning under time pressure.

## Consequences
- The final choice of local planner is not fixed today; it is recorded as an update to this ADR's Status once the Phase 3 benchmark completes.
- Both configurations must be kept buildable until the benchmark is run, which adds a small amount of Phase 3 setup overhead (two parameter files instead of one).
- Because the decision criterion is numeric and pre-registered, it cannot be re-litigated after the fact based on preference.

## Alternatives Rejected
- **Pick DWB outright without benchmarking:** rejected — the course brief requires the local planner choice to carry a benchmark-based selection criterion, not a default-of-convenience.
- **Pick TEB outright without benchmarking:** rejected for the same reason, and because TEB's additional tuning surface is a schedule risk if selected without evidence that it outperforms DWB in this scenario.
- **Implement a custom local planner:** rejected — Nav2 is to be configured, not rebuilt (PRD NG3).
