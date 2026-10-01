# ADR-004: Local Planner: DWA vs TEB, Chosen by Benchmark

**Status:** Accepted · the final pick is recorded here after P3

## Context
M3 (moving NPCs) depends mostly on the local planner. Noetic ships two well-supported options:

| | DWA (`dwa_local_planner`) | TEB (`teb_local_planner`) |
|---|---|---|
| How | Samples velocities, scores short trajectories | Optimises a time-aware elastic band |
| Strength | Simple, fast, TurtleBot3 default | Smoother around moving obstacles |
| Cost | Can hesitate with crossing obstacles | More parameters to tune |

## Decision
- **P1–P2 use DWA** (TurtleBot3 defaults as the starting point).
- **In P3, run both on S-04** with N = 20 each (`--local-planner dwa|teb`).
- **Pick the winner by these rules, in order:**
  1. Fewer total collisions.
  2. If tied, higher success rate.
  3. If still tied, DWA (less tuning risk).
- **Time box: 2 days.** If the benchmark is not finished, keep DWA.

## Consequences
- Two config files (`dwa_local_planner.yaml`, `teb_local_planner.yaml`) are kept working until P3 ends.
- `motion_profile_node` must know the active planner's `dynamic_reconfigure` namespace.
- The rule is fixed now, so the choice cannot be argued afterwards.

## Result
| Planner | Success | Collisions | Chosen |
|---------|:-------:|:----------:|:------:|
| DWA | _tbd_ | _tbd_ | |
| TEB | _tbd_ | _tbd_ | |
