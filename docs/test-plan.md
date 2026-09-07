# Test Plan — UC6 Mobile Robot Warehouse System

## 1. Metric Definitions

| Metric | Unit | Definition / measurement method |
|--------|------|----------------------------------|
| Collisions per run | count | Number of `collision` `run_events` logged in the run (Unity collider contact between robot footprint and any obstacle/wall/robot). |
| Goal success rate | % over N≥20 runs | (count of runs with `outcome = success`) / N × 100. |
| Minimum obstacle clearance | metres | `MIN()` over all `clearance_sample` events in the run; each sample is the local-costmap-derived distance from the robot footprint boundary to the nearest obstacle, taken at 2 Hz. |
| Path length ratio | dimensionless | `path_length_m / straight_line_baseline_m`, where `path_length_m` is the integrated odometry travel distance and `straight_line_baseline_m` is the Euclidean distance between the task's start and goal poses. |
| Time-to-goal | seconds | `end_time − start_time` for a `success` outcome, where `start_time` is task dispatch and `end_time` is the goal-reached event. |
| Replan count per run | count | Number of `replan` `run_events` logged in the run. |
| Mean linear velocity per category | m/s | Mean of the commanded `/cmd_vel` linear-x samples over a run, grouped by the task's item category across all runs in a scenario, computed by the CSV exporter (not stored per-run in `runs`, since it is a cross-run rollup). |

## 2. Scenario Matrix

| ID | Scenario type | Description | Milestone | Acceptance criterion |
|----|-----------------|--------------|:--:|------------------------|
| S-00 | Baseline (M1) | Open room, no obstacles, single robot, Standard category only | M1 | Goal success rate ≥ 90% over N=20; 0 collisions |
| S-01 | Static-only | Single static obstacle placed directly between shelf and drop-off poses | M2 | 0 collisions across 20 consecutive runs; min clearance ≥ 0.15 m |
| S-02 | Narrow-corridor (static) | Corridor width 1.5× robot footprint, static obstacles along both sides | M2 | 0 collisions across 20 consecutive runs; min clearance ≥ 0.15 m; path length ratio ≤ 1.3 |
| S-03 | Dead-end recovery (static) | A dead-end branch forces at least one recovery-behaviour trigger before the correct path is found | M2/M3 | 0 collisions across 20 consecutive runs; goal success rate ≥ 90% over N=20 (recovery must resolve, not just avoid collision) |
| S-04 | Dynamic-NPC | Scripted NPC crosses the robot's path at a fixed interval | M3 | Goal success rate ≥ 85% over N=20; ≤1 collision per 20 runs; replan count > 0 on at least 1 run in 20 |
| S-05 | Multi-robot (Stage B/C) | 2 (Stage B) or 3 (Stage C) independently-navigating robots sharing the warehouse map | Beyond M3 (stretch, gated) | 0 cross-robot TF/namespace conflicts across 20 consecutive runs; goal success rate ≥ 80% over N=20 for every robot in the stage |
| S-06 | Per-category profile | Fixed route (open room, no extraneous obstacles), 20 runs per category (Fragile / Standard / Heavy), 60 runs total | Ungraded (proves FR-09) | Mean linear velocity per category strictly ordered Fragile < Heavy < Standard, with each pairwise difference ≥ 0.03 m/s |

All "20 consecutive runs" / "N=20" figures are the minimum required by the acceptance-criteria rules (N≥20); the headless runner (§3) is what makes running this volume of trials for every scenario and every planner candidate (ADR-004) practical within the 8-week timeline.

## 3. Headless Automated Runner Specification

- **Invocation:** one runner process per scenario, parameterised by `scenario_id` and `run_count` (default 20).
- **Per-run sequence:** (1) ensure MySQL is up and seeded with a `pending` task matching the scenario's category/route; (2) launch/reset the Unity scene in batch/headless mode for that scenario [VERIFY Unity batch-mode compatibility with the ROS-TCP bridge in the pinned Unity version — Phase 0 task]; (3) launch the ROS 2 launch file for the scenario's robot count (1 for S-00–S-04/S-06, 2 or 3 for S-05); (4) dispatch the task via `task_manager`/`mission_orchestrator` as in normal operation (`docs/diagrams.md` (b)); (5) wait for episode end (success, fail, or a hard timeout, e.g. 3× the S-00 median time-to-goal); (6) confirm the `runs`/`run_events` rows were written (or the disk-buffer fallback triggered, per FR-16); (7) reset scene state for the next run.
- **Isolation:** each run uses a fresh `task` row so that `runs.task_id` unambiguously identifies the trial; the seed script's re-seed mode (`docs/data-model.md` §5) is used between scenario batches, not between individual runs, to keep runtime reasonable.
- **Output:** on completion of a scenario's `run_count` runs, the runner invokes the CSV exporter (§4) against that scenario's `runs`/`run_events` rows.

## 4. CSV Output Schema

One row per run, one file per scenario (`<scenario_id>_results.csv`):

| Column | Source |
|--------|--------|
| `run_id` | `runs.run_id` |
| `scenario_id` | `runs.scenario_id` |
| `task_id` | `runs.task_id` |
| `robot_namespace` | `runs.robot_namespace` |
| `item_category` | joined from `tasks.item_id → items.category_id → categories.name` |
| `outcome` | `runs.outcome` |
| `collisions_count` | `runs.collisions_count` |
| `min_clearance_m` | `runs.min_clearance_m` |
| `path_length_m` | `runs.path_length_m` |
| `straight_line_baseline_m` | `runs.straight_line_baseline_m` |
| `path_length_ratio` | `runs.path_length_ratio` |
| `time_to_goal_s` | `runs.time_to_goal_s` |
| `replan_count` | `runs.replan_count` |
| `mean_linear_vel_mps` | `runs.mean_linear_vel_mps` |
| `start_time` | `runs.start_time` |
| `end_time` | `runs.end_time` |

A second, scenario-level summary file (`<scenario_id>_summary.csv`) reports the aggregate values referenced in §2's acceptance criteria (goal success rate, collisions per 20 runs, min-of-mins clearance, mean path length ratio, and — for S-06 only — mean linear velocity per category).

## 5. Scripted Manual Demo Sequence (Grading Day)

| Step | Action | Purpose |
|:--:|--------|---------|
| 1 | Start `docker-compose up` (MySQL) and confirm the seeded catalog is present | Show the persistence layer is live and independent of the sim |
| 2 | Launch the Unity scene in Play mode | Visual context for the grader |
| 3 | Launch the ROS 2 stack via the project's single launch file | Show the full node graph coming up cleanly |
| 4 | Run scenario S-02 (narrow corridor, Standard item) live, narrating the global/local planner behaviour | Demonstrates M2 |
| 5 | Run scenario S-04 (dynamic NPC) live, narrating the re-plan trigger when the NPC crosses | Demonstrates M3 |
| 6 | Run scenario S-06 back-to-back for all three categories, narrating the visibly different speeds | Demonstrates FR-09 without over-emphasising it as a standalone feature |
| 7 | If the Stage B gate was met, run one Stage B (2-robot) episode | Demonstrates the multi-robot stretch, time permitting |
| 8 | Display the Phase 5 CSV summary tables for S-00–S-06 | Presents the numeric N≥20 evidence backing every claim above |
| 9 | If any live run misbehaves, fall back to the pre-recorded Phase 5 headless run logs/CSVs for that scenario | Keeps the demo from being derailed by simulation flakiness on the day |
