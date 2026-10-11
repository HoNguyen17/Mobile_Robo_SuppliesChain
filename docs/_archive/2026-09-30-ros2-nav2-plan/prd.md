# PRD — UC6 Mobile Robot Warehouse System

**Course:** 61CSE326 · **Use case:** UC6 (Shopee-style e-commerce fulfilment, simulated in Unity)
**Team size:** 4 · **Timeline:** 8 weeks · **Readers:** course graders, implementing team

---

## 1. Problem Statement

A simulated warehouse must fulfil e-commerce pick-and-place tasks using an autonomous mobile robot while avoiding static and dynamic obstacles. Grading centres on the robot's obstacle-avoidance competence, demonstrated across three milestones (M1 tutorial adaptation, M2 static avoidance, M3 dynamic avoidance). Item identification and any perception-driven classification are course-mandated non-graded scaffolding and must not consume disproportionate design or implementation effort.

## 2. Goals

| # | Goal |
|---|------|
| G1 | Demonstrate a working Nav2-based navigation stack adapted from the Unity Robotics Hub tutorial (M1). |
| G2 | Demonstrate quantifiably safe navigation around static obstacles (M2), with binary pass/fail metrics. |
| G3 | Demonstrate quantifiably safe navigation around dynamic obstacles, including scripted NPCs and (where staged gates permit) other robots (M3). |
| G4 | Reinforce obstacle-avoidance behaviour with category-aware motion profiles (fragile / standard / heavy), proven via measurable velocity differences — not presented as a standalone feature. |
| G5 | Persist task and telemetry data reliably without that persistence ever gating or blocking a simulation run. |
| G6 | Produce reproducible, numeric evidence (headless automated runs + a rehearsed manual demo) suitable for grading. |

## 3. Explicit Non-Goals

| # | Non-goal | Rationale |
|---|----------|-----------|
| NG1 | ML-based item/shelf detection (e.g. YOLOv8n) in the graded path | Perception is ungraded; known shelf-slot coordinates are used instead (ADR-002). |
| NG2 | Arm-based grasping or manipulation planning | TurtleBot3 Waffle Pi carries no arm; pickup is simulated via dwell + re-parent (ADR-001). |
| NG3 | Rebuilding or replacing Nav2 internals | Nav2 is configured, not rebuilt. |
| NG4 | C++ nodes or tooling | Python (ROS 2) and C# (Unity) only. |
| NG5 | External diagramming tools | Mermaid, inline in markdown, only. |
| NG6 | Treating item classification as a first-class feature | It exists only to parameterise motion profiles; see G4. |

## 4. Actors

| Actor | Description |
|-------|-------------|
| Warehouse Robot (TurtleBot3 Waffle Pi) | Navigates to a shelf slot, dwells, "carries" the re-parented item, navigates to drop-off, dwells, releases. |
| Scripted NPC Obstacle | Unity-driven moving obstacle used from M3 / Stage A onward; not a ROS node. |
| Second / Third Robot | Independently navigating robots introduced at multi-robot Stage B / Stage C. |
| Mission Orchestrator (ROS 2 node) | Drives the per-task navigation/pickup/drop-off state machine. |
| Motion Profile Node (ROS 2 node) | Maps item category to Nav2 runtime parameters. |
| Task Manager (ROS 2 node) | The single node permitted to touch MySQL; reads the task at episode start, writes telemetry at episode end. |
| Metrics Collector (ROS 2 node) | Aggregates in-run events (collisions, replans, clearance samples) and hands a summary to Task Manager. |
| Course Grader | Consumes milestone evidence: automated metrics + live/scripted demo. |
| Implementing Team (4 students) | Builds and integrates the system against this PRD. |

## 5. Functional Requirements

| ID | Requirement | Notes |
|----|-------------|-------|
| FR-01 | Task Manager shall read the next pending task (item, drop-off pose) from MySQL at episode start. | If MySQL is unreachable, fall back per FR-16. |
| FR-02 | Mission Orchestrator shall command Nav2 (`navigate_to_pose`) to the item's known shelf-slot pose. | Shelf pose sourced from catalog, not perception (ADR-002). |
| FR-03 | Nav2's global planner shall produce a collision-free path to the shelf pose in the presence of static obstacles. | Graded: M2. |
| FR-04 | Nav2's local planner/controller shall track the global path while avoiding static obstacles not present in the static map. | Graded: M2. |
| FR-05 | Nav2's local planner/controller shall avoid dynamic obstacles (scripted NPCs, other robots) that intrude on the planned path. | Graded: M3. |
| FR-06 | On planner/controller failure, Nav2 recovery behaviours (costmap clear, spin, back-up, wait) shall execute before an abort is declared. | Graded: M2/M3. |
| FR-07 | On dynamic obstacle intrusion, the system shall trigger a global re-plan and record a `replan` event. | Graded: M3. |
| FR-08 | On arrival within the shelf-pose tolerance, the robot shall dwell for a configured duration, after which Unity shall re-parent the item GameObject to the robot. | Ungraded (mechanical support). |
| FR-09 | Before departing the shelf, Motion Profile Node shall apply the Nav2 parameter set for the item's category (fragile / standard / heavy). | Reinforces M2/M3 — not a standalone feature (G4). |
| FR-10 | Mission Orchestrator shall command Nav2 to the task's drop-off pose with the category profile active, subject to the same static/dynamic avoidance requirements as FR-03–FR-07. | Graded: M2/M3. |
| FR-11 | On arrival at the drop-off pose, the robot shall dwell for a configured duration, after which Unity shall release (un-parent) the item GameObject. | Ungraded (mechanical support). |
| FR-12 | Metrics Collector shall record, for every run: collision events, replan events, periodic clearance samples, and the final goal outcome. | Ungraded (verification tooling), evidences M1–M3. |
| FR-13 | Task Manager shall write one `runs` row and its associated `run_events` rows to MySQL in a single transaction at episode end. | Ungraded (infra); see ADR-005. |
| FR-14 | A MySQL outage (at start or end of episode) shall never abort or block the simulation run. | Ungraded (reliability); see FR-16. |
| FR-15 | The item catalog shall provide category, shelf-slot pose, and (optionally) weight for every item; no runtime ML inference is used to obtain these. | Ungraded; see ADR-002. |
| FR-16 | On MySQL read failure at episode start, Task Manager shall fall back to the last-cached task list (or a bundled default task) and log a warning; on write failure at episode end, it shall buffer the record to local disk and retry on the next episode start. | Ungraded (reliability). |
| FR-17 | Stage A shall run exactly one robot with scripted NPC obstacles only. | Graded: M3 baseline. |
| FR-18 | Stage B shall add one independently-navigating second robot under its own namespace/TF prefix, gated on the Stage A exit criteria. | Beyond M3 (stretch, gated); see ADR-006. |
| FR-19 | Stage C shall add a third independently-navigating robot, gated on the Stage B exit criteria. | Beyond M3 (stretch, gated); see ADR-006. |
| FR-20 | A headless automated runner shall execute each test scenario N≥20 times and export a CSV of per-run metrics. | Ungraded (verification tooling), evidences M1–M3. |
| FR-21 | YOLOv8n-based shelf/item detection may be prototyped only after the Stage/M3 exit gates are met, and shall never sit on the graded navigation path. | Explicitly out-of-scope stretch (NG1). |

## 6. Non-Functional Requirements

| ID | Requirement |
|----|-------------|
| NFR-01 | All ROS 2 nodes are implemented in Python; all Unity scripts are implemented in C#. No C++. |
| NFR-02 | The Nav2 local planner (DWB or TEB) is selected by a benchmark-based criterion, not by preference (ADR-004). |
| NFR-03 | Exactly one ROS 2 node (`task_manager`) performs MySQL I/O; no planner or control-loop node queries the database at runtime. |
| NFR-04 | The system tolerates a MySQL outage without any change to navigation behaviour (FR-14/FR-16). |
| NFR-05 | All diagrams are Mermaid, inline in markdown; no external diagramming tool output is checked in. |
| NFR-06 | The ROS 2 distribution is pinned as a Phase 0 output, not assumed; see ADR-003 and the [VERIFY] list. |
| NFR-07 | Category motion-profile parameters are hot-applicable per task without restarting Nav2 nodes. |
| NFR-08 | Multi-robot namespacing/TF-prefixing introduces no cross-robot topic collisions at any staged level (A/B/C). |
| NFR-09 | Every acceptance criterion is binary pass/fail with an attached number; no prose criteria (see §8). |

## 7. Traceability Table

| Requirement | M1 | M2 | M3 | Ungraded |
|-------------|:--:|:--:|:--:|:--------:|
| FR-01 | ✔ | ✔ | ✔ | |
| FR-02 | ✔ | ✔ | ✔ | |
| FR-03 | | ✔ | | |
| FR-04 | | ✔ | | |
| FR-05 | | | ✔ | |
| FR-06 | | ✔ | ✔ | |
| FR-07 | | | ✔ | |
| FR-08 | | | | ✔ |
| FR-09 | | ✔ | ✔ | |
| FR-10 | | ✔ | ✔ | |
| FR-11 | | | | ✔ |
| FR-12 | | | | ✔ |
| FR-13 | | | | ✔ |
| FR-14 | | | | ✔ |
| FR-15 | | | | ✔ |
| FR-16 | | | | ✔ |
| FR-17 | | | ✔ | |
| FR-18 | | | | ✔ (stretch) |
| FR-19 | | | | ✔ (stretch) |
| FR-20 | | | | ✔ |
| FR-21 | | | | ✔ (stretch) |
| NFR-01–NFR-09 | — | — | — | — (cross-cutting; see §6) |

## 8. Success Metrics

All acceptance criteria are numeric and binary; see `docs/test-plan.md` for full scenario-by-scenario values. The defined metrics are:

| Metric | Unit | Definition |
|--------|------|------------|
| Collisions per run | count | Contact events between robot footprint and any obstacle/wall/robot during one run. |
| Goal success rate | % over N≥20 runs | (runs reaching goal within pose tolerance) / N × 100. |
| Minimum obstacle clearance | metres | Smallest recorded distance from robot footprint boundary to nearest obstacle during the run. |
| Path length ratio | dimensionless | Actual travelled path length ÷ straight-line (Euclidean) start-to-goal distance. |
| Time-to-goal | seconds | Elapsed time from task dispatch to goal-reached event. |
| Replan count per run | count | Number of global re-plans triggered during the run. |
| Mean linear velocity per category | m/s | Mean commanded linear velocity over a run, grouped by item category, used to prove FR-09 measurably changes robot behaviour. |

Example of the required criterion phrasing (see `docs/test-plan.md` for the full matrix):
> "0 collisions across 20 consecutive runs in scenario S-02 (narrow corridor, static obstacles)."

Milestone-level gates (headline numbers; full matrix in `docs/test-plan.md`):

| Milestone | Headline pass condition |
|-----------|--------------------------|
| M1 | Goal success rate ≥ 90% over N=20 in scenario S-00 (open room, no obstacles); 0 collisions. |
| M2 | 0 collisions across 20 consecutive runs in scenario S-02; minimum clearance ≥ 0.15 m; path length ratio ≤ 1.3. |
| M3 | Goal success rate ≥ 85% over N=20 in scenario S-04 (dynamic NPC); ≤1 collision per 20 runs; replan count recorded and non-zero on at least one intrusion event per run. |

Basis for the above thresholds: 0.15 m clearance reflects the Waffle Pi footprint radius (~0.22 m half-diagonal) plus a conservative margin against the default Nav2 inflation radius; the M3 collision tolerance (≤1/20) reflects that dynamic-obstacle avoidance depends on reactive local-planner tuning rather than a fully deterministic global plan, unlike the static case. These are tunable defaults — see the risk register in `docs/milestones.md` for the process to revise them after Phase 2/3 dry runs.
