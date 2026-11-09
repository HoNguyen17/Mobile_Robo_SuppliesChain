# Changelog

All notable changes to this repository are documented in this file, newest first. The format follows [Keep a Changelog](http://keepachangelog.com/en/1.0.0/). There are no version tags, so entries are grouped by date. The history was written from `git log` on 2026-11-09; commit hashes are in brackets.

The project is the UC6 warehouse robot: a simulated TurtleBot3 Waffle Pi in Unity (URP) driven by our own ROS 1 Noetic Python navigation nodes. Everything before 2026-09-08 is the upstream Unity warehouse template (see the end of this file).

## Unreleased

### Fixed
- Planners: plans no longer overlap, a replan no longer blocks the scan callback, and `/nav/cancel` makes both planners forget the goal and the path, so a stale path is not republished after a leg ended. One `PlanScheduler` thread plans; stale results are dropped ([ADR-016](docs/ADR-016-planner-scheduling.md)). Closes open issues 10, 18 and 19; replan timing can differ slightly from the earlier benchmarks.

- The live `/costmap` uses the inflation radius of the plan being followed (`nav_common.costmap_radius`), so RViz no longer alternates between 0.40 m and 0.35 m ([ADR-017](docs/ADR-017-live-costmap-radius.md)). Closes open issue 11.

### Added
- `scripts/plan_scheduler.py` and tests `test_plan_scheduler.py`, `test_planner_nodes.py` (both real planner nodes against a fake rospy), `test_planner_scheduling_source.py`; `plan_scheduler.py` is installed by `CMakeLists.txt`.
- Exact two-sided permutation test on means in `ab_report.py` (`permutation_test_means`), with a per-scenario table in `report.md` and `report.html` and 17 tests in `test_ab_tools.py`.
- `docs/benchmark-data/20261026_214123/`: tracked copy of the parity batch (CSVs, `meta_*.json`, both reports, `handoff.txt` with personal paths redacted).
- `README-BENCHMARK.md` section 7.5: how to reproduce the p-values, and which values of the pooled table do not reproduce from the CSVs.
- `mission_node.py` labels its run summaries from `~planner_name`, then the global `/nav/planner_name`, then `astar`. `bringup_dstar.launch` sets `/nav/planner_name` to `dstar` [af59c3e]; `bringup.launch` sets it to `astar` when it starts the A\* planner (`nav:=true`) ([ADR-018](docs/ADR-018-planner-label.md)). Closes open issue 14.
- `run_scenario.py` and `run_all_scenarios.py` log "PLACEHOLDER: results are not measured". Their results are still hardcoded.

### Changed
- `turtlebot_control` declares `sensor_msgs` (it imports `LaserScan`) and installs `dstar_planner.py` and `dstar_lite.py`. `catkin_make` in `~/nav_ws` builds all five packages with this.
- Comments and docs state the real values: footprint 0.257 m (ADR-015), not 0.21 m; the follower's `~final_tol` is 0.20 m while `PathTracker`'s default is 0.10 m. No behaviour changed.
- `RUN_STEPS.txt` and the main `README.md` no longer quote test counts [cc21f5f].
- `README-BENCHMARK.md`: the pooled-table cells for planner time and share planning no longer claim "p < 0.001"; only 5 vs 5 runs carry those columns, so p = 0.008 [3ec9ee2].
- `docs/LOOP_PROGRESS.md` carries a "historical" banner; its text is unchanged [546a775].

### Removed
- `docs/ADR-003-ros-distro-pinning.md`, `docs/ADR-004-nav2-local-planner.md` and `docs/ADR-006-multirobot-staging.md`: byte-identical to the copies in `docs/_archive/2026-09-30-ros2-nav2-plan/` and sharing numbers with the live ADR-003 and ADR-006 [4f35c84].
- The broken `Mobile_Robo_SuppliesChain` gitlink (no `.gitmodules`, empty directory) [3441a44].

## 2026-11-09

### Changed
- Context split into `CLAUDE.md`, `CONTEXT.md` and `README-BENCHMARK.md` [7978f2e].

## 2026-10-27

### Added
- Documentation of the fair-parity benchmark results (batch `20261026_214123`) and the verified test count [c9f2d3f, 276a7af].

## 2026-10-26

### Added
- D\* Lite debug options: `~filter_static_hits`, `~trigger_debug`, `~obstacle_memory_s`, and the matching `ab_batch.py` flags [f9c73f4].

### Fixed
- `astar_planner` now perceives obstacles and checks the path for replanning the same way as `dstar_planner` (static-return filter, adaptive check radius, obstacle memory), so A\* and D\* Lite can be compared fairly. Before this, A\* replanned about half as often as D\* only because it kept static wall returns [3a1ab03].

## 2026-10-25

### Added
- D\* Lite planner (`dstar_planner.py`, `dstar_lite.py`, `navigation_dstar.launch`, `bringup_dstar.launch`) next to the A\* planner [51582f7].
- One-click A/B benchmark: `ab_run.py`, `ab_batch.py`, `ab_report.py`, `benchmark_planners.py`, and the Unity menus `Warehouse > A-B Test` and `1D. Launch ROS System with D* Lite` [51582f7].
- How-to for the benchmark and the D\* Lite demo [930edc9].

### Fixed
- Scenario reset in Unity is reliable enough for automated runs: the `/sim/ack` publisher is registered in `Start()`, the root `ArticulationBody` is found robustly, and link velocities are zeroed on reset [ece7d7b].

## 2026-10-12

### Added
- The complete warehouse system on branch `nguyen`, merged into `main` [dee4b6d, 91c451b]: dynamic obstacle avoidance in the planner and follower, `NpcMover` (crossing NPC), `ScenarioLoader`, `ItemCarrier`, `PathVisualizer`, the one-click Unity launch menu, and a rewritten README.

## 2026-10-05

### Added
- `turtlebot_control` navigation package (A\* planner, path follower, pure-Python modules, 109 unit tests with a closed-loop Waffle Pi model) and `warehouse_bringup` with a one-line launch [3c5f29a].
- Physical Waffle Pi in Unity at scale 4 with `/robot/pose`, `/map` and `/sim/collision` publishers (ADR-015) [2bf8221, 9853320].

### Changed
- The cube prototypes moved to `_archive/` [3c5f29a, 2bf8221]. Test Runner vs ROS explained in the `turtlebot_control` README [bd1065c]. Local Unity test run logs are git-ignored [df82131].

## 2026-10-04

### Added
- TurtleBot3 Waffle Pi as a scaled visual model on the cube [f41e0b6, 3f0f7bd].
- ADR-012 to ADR-014: custom Python navigation, ROS interfaces; plan documents re-aligned with the code base [3133092, 2b4a18f, f5af9e5, ffe5411].
- `cube_control` planner and follower fixed, with leg result and cancel topics; logic moved into modules without rospy, 85 unit tests [47e7a67].
- Merged `.gitattributes` and `.gitignore` for the `ros1/` layout [c6869c2].

## 2026-10-01

### Added
- ROS-TCP-Connector and URDF-Importer packages, Unity ROS publishers and subscribers, the occupancy-grid publisher [341b010, 303e66b].
- ROS 1 `cube_control` package (A\* planner, path follower, tools) and its README [d2ef2ca, 6e840dc].
- LF line-ending rules for ROS files and ignore rules for ROS artifacts [a215a7f].

## 2026-09-25

### Added
- Unity prototype: grid-based A\* navigator for a cube [4a8eea2].

## 2026-09-08

### Added
- Repository started on top of the upstream Unity warehouse template, with the first project documents [034e1ca, 50f4e36].

## Upstream Unity warehouse template (before 2026-09-08)

The text below is the changelog that came with the upstream template, without its empty headings.

### v0.0.1

#### Upgrade Notes

Miscellaneous warehouse generation fixes, updated menu option to copy template scene.

Updated documentation for new workflows.

HDRP and URP package separation.
