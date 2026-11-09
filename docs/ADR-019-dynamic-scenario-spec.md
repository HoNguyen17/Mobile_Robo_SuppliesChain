# ADR-019: Dynamic Scenarios Are Data, and the Reset Message Can Carry a Seed

**Status:** Accepted (2026-11-09) · milestone 3 · changes the benchmark protocol (the old protocol stays available) · follows [ADR-016](ADR-016-planner-scheduling.md)

## Context
Milestone 3 needs more dynamic-obstacle scenarios than the one hardcoded crossing worker of S-04 (`ScenarioLoader.cs`), and N = 20 runs that are not near-replicates:
- The S-04 worker moves from the start of the reset on a fixed line at 0.5 m/s, and the robot leaves 7 s later, so every run meets the worker at the same phase.
- A red draggable box (`Dynamic_Draggable_Obstacle`, Unity (2.5, 1.5, 10)) is created at `Start()` and is never removed by a reset. Whether it affected earlier results is not known (step U2).
- Adding a scenario meant editing C# and scenes, and every edit costs a Unity compile and manual setup.

## Decisions
1. **One source, `ros1/warehouse_eval/config/scenarios.yaml`.** A scenario with a `workers:` list is *spec-driven*. A worker has a name, speed, radius, a list of waypoints (ROS metres, absolute or relative to the planned straight route as `{f, offset_m}`), per-waypoint `dwell_s` (-1 = for ever), mode `once` (holds the last waypoint) or `pingpong`, a start trigger and `start_delay_s` + `start_jitter_s`.
2. **Export, not parsing in Unity.** `python3 -m warehouse_eval.scenario_spec --write` resolves the yaml (route-relative points become absolute) into `WarehouseProjectURP/Assets/StreamingAssets/scenarios.json`. Unity reads that file at every seeded reset, so a new scenario needs no recompile. A test fails while the JSON is stale or the yaml is invalid.
3. **The reset message is `ID` or `ID?seed=N`** on the existing `std_msgs/String` topic `/sim/reset_scenario`. **An ID without `?` takes exactly the old code path**: the hardcoded scenarios, the draggable box present, the old S-04 worker. A malformed message, an unknown ID or a missing/invalid `scenarios.json` on the seeded path is answered with `/sim/ack` `ok:false` and an error text; it never falls back silently.
4. **Seeded start offset.** A worker with `start_trigger: robot_departs` is absent from the scene until the robot has moved 0.3 m from its start pose, plus `start_delay_s + start_jitter_s * seeded_unit(seed, worker index)`; then it appears at its first waypoint. `seeded_unit` is a 32-bit integer hash with the same code in Python and C# and pinned test vectors. `ab_run` uses `seed = seed_base + run_index`, so run k of A\* and run k of D\* meet the same worker timing (paired runs).
5. **Isolation on the seeded path only.** The draggable box and the manual 'O' / '0' / 'B' box are deactivated (flag `isolateSeededScenarios`, default true); a plain reset activates the draggable box again. A scenario is therefore the only dynamic thing in the scene while a seeded run is measured.
6. **Ground truth for metrics, never for control.** Unity publishes the active workers on `/sim/npc_poses` (`geometry_msgs/PoseArray`, frame `map`, 20 Hz). Only `ab_run.py` may subscribe to it (worker gap, `collisions_npc`). A test fails if the planners, the follower or any other script subscribe to it.
7. **Record the protocol.** The seed goes into the CSV (`seed`) and into `meta_*.json`. Data recorded without a seed (the 2026-10-26 parity batch) used the old protocol and is not comparable with seeded data.

## Consequences
- The old S-04 line is kept in the spec (absolute waypoints equal to the C# line), so the seeded S-04 measures the scenario that was benchmarked, with a varying start time and without the draggable box. `docs/test-plan.md` says "every 8 s"; the old code had no pause and a 6 s leg, and the spec does not either.
- Worker geometry for the new scenarios is route-relative and **unverified**: the aisle layout was not looked at in Unity, so a worker may start inside a rack. `layout_checked: false` in the yaml marks this; the matrix runner prints a warning for each such scenario until the layout check (U3) is done and the flag is set.
- A worker that is hidden before it starts is not seen by the lidar before then; it appears at its first waypoint, which can be inside the lidar range. This is deliberate (an aisle worker that stands for 20 s before the robot arrives would be a static obstacle in the first plan) and is a modelling choice to be reviewed with the baseline.
- Worker dwell times are fixed per scenario; only the start time is seeded.
- Tests: `ros1/warehouse_eval/test/test_scenario_spec.py` (reset message, seeded values, path maths, validation, export, JSON freshness, one test-plan row per scenario); `NpcPathTests.cs` (EditMode, same vectors; run by hand, see the U1 steps); `test_ab_tools.py` (seed in the reset message and the CSV, the subscription rule).
- Not verified without Unity: that the C# code builds, that workers move as specified, and that Unity detects contacts with a kinematic worker at 1.0 m/s.
