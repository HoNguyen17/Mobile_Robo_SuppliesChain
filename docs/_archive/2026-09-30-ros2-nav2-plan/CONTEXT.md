# CONTEXT — UC6 Mobile Robot Warehouse System

The glossary and domain overview for this repo. When naming a domain concept in an issue, a test name, a commit, or a code identifier, use the term as defined here.

Companion documents: `docs/prd.md` (requirements), `docs/architecture.md` (node graph, bridge contract), `docs/data-model.md` (schema), `docs/test-plan.md` (metrics, scenarios), `docs/milestones.md` (phases, gates), `docs/ADR-NNN-*.md` (decisions).

---

## 1. Domain Overview

A **simulated** warehouse fulfils e-commerce pick-and-place tasks with an autonomous mobile robot. Unity owns the world (scene, robot body, obstacles, item GameObjects); ROS 2 owns the thinking (navigation, task sequencing, telemetry); the two talk over a TCP bridge. MySQL holds the catalog and the evidence.

The system exists to demonstrate **obstacle avoidance**. Everything else — item identification, pickup mechanics, the database — is scaffolding that must never grow beyond its supporting role.

---

## 2. The Three Axes (do not conflate)

These three words each mean something different and are the most common source of confusion.

| Axis | Values | What it measures | Defined in |
|------|--------|------------------|------------|
| **Milestone** | M1, M2, M3 | What the *course grades*: M1 tutorial adaptation, M2 static avoidance, M3 dynamic avoidance | `docs/prd.md` |
| **Phase** | Phase 0–5 | Where the *team is in the 8-week calendar* | `docs/milestones.md` |
| **Stage** | A, B, C | How many *robots* run concurrently: 1, 2, 3 | `docs/architecture.md` §7, ADR-006 |

Stage A is the M3 baseline. Stages B and C are gated stretch work beyond M3.

---

## 3. Core Vocabulary

### Work and its record

**Task** — a unit of fulfilment work: one item plus a drop-off pose. Lives in the `tasks` table with status `pending → in_progress → completed | failed`. A task is *requested* work, not an attempt at it.

**Episode** — one live attempt at a task, from task dispatch to final outcome. The runtime word. An episode begins when `task_manager` hands a task to `mission_orchestrator` and ends when the outcome is reported. One task may be attempted over several episodes.

**Run** — the persisted record of one episode: a `runs` row plus its `run_events`. The data word. *Episode* is the thing happening; *run* is the row it leaves behind. Use "run" when talking about metrics, CSV export, or N≥20 counts; use "episode" when talking about the live sequence.

**Outcome** — the terminal result of an episode: `success` or `fail`. Not "result", not "status" (`status` belongs to `tasks`).

### The item catalog

**Item** — a catalog entry with a category, a home shelf slot, and an optional weight. Item identity is known from the catalog, never inferred at runtime (ADR-002).

**Category** — exactly one of **Fragile**, **Standard**, **Heavy**. Every item belongs to one. Categories exist solely to parameterise the motion profile — never call category assignment "classification" or "detection", which imply perception the system deliberately does not do.

**Motion profile** — the Nav2 parameter set (`max_vel_x`, `acc_lim_x`, `inflation_radius`) a category maps to, applied by `motion_profile_node` before the drop-off leg. Framed as *reinforcing obstacle avoidance*, not as a standalone feature (PRD G4).

**Shelf slot** — a known static pose (`pose_x`, `pose_y`, `pose_theta`) where an item lives. Catalog-sourced coordinates stand in for perception (ADR-002).

**Drop-off pose** — the task's destination pose. The second navigation leg's goal.

### The robot's sequence

**Leg** — one `navigate_to_pose` goal. An episode has two: the *shelf leg* and the *drop-off leg*.

**Dwell** — the configured pause on arriving within pose tolerance, before pickup or release. The robot is stationary and the simulation is waiting, deliberately.

**Re-parent** — Unity's simulated pickup: the item GameObject is attached to the robot so it travels with it. **Release** (or *un-parent*) is the inverse at drop-off. There is no grasping and no arm — the Waffle Pi has neither (ADR-001). Never call this "grasp" or "manipulate".

### Navigation and safety

**Static obstacle** — an obstacle not present in the static map but not moving. Graded at M2.

**Dynamic obstacle** — a moving obstacle: a scripted NPC (Stage A) or another robot (Stage B/C). Graded at M3.

**Scripted NPC** — a Unity-driven moving obstacle. It is *not* a ROS node and has no navigation stack of its own. Distinct from a *second/third robot*, which navigates independently.

**Clearance** — the local-costmap-derived distance from the robot footprint boundary to the nearest obstacle, sampled at 2 Hz. "Minimum clearance" is the per-run minimum over those samples.

**Replan** — a global re-plan triggered by dynamic obstacle intrusion, logged as a `replan` event. Evidence that avoidance is working, not a failure.

**Recovery behaviour** — a stock Nav2 response to a stuck or failed plan: spin, back-up, wait, clear costmap. Recovery precedes any abort.

**Collision** — a Unity collider contact between the robot footprint and any obstacle, wall, or other robot, logged as a `collision` event. The primary thing the grade punishes.

### Verification

**Scenario** — a defined test configuration with a numeric acceptance criterion, identified `S-00` through `S-06` in `docs/test-plan.md`. Scenarios, not opinions, decide whether a milestone is met.

**Gate** — a numeric exit condition that must be met before the next phase or stage begins. Gates are pass/fail and are never waived silently; a failed stage gate means shipping the prior stage (ADR-006).

**Headless runner** — the automated harness that executes a scenario N≥20 times without a visible Unity window and exports per-run metrics to CSV. Produces the grading evidence.

**Graded path** — the code path the course actually scores: navigation and obstacle avoidance. Work that is not on the graded path (perception, database, pickup mechanics) is capped in effort by design and must never be allowed to block or gate the graded path.

### Integration

**Bridge** — the Unity `ROS-TCP-Connector` ↔ ROS 2 `ros_tcp_endpoint` TCP link. One connection per robot instance. The message contract is in `docs/architecture.md` §2.

**Namespace / TF prefix** — the per-robot isolation scheme (`/robot1`, `robot1/`) that lets multiple Nav2 stacks coexist. Collisions here are *namespace conflicts*, never to be confused with physical *collisions*.

---

## 4. Node Responsibilities in One Line Each

| Node | One-line responsibility |
|------|--------------------------|
| `mission_orchestrator` | Runs the per-task FSM: shelf leg → dwell → profile → drop-off leg → dwell → report. |
| `motion_profile_node` | Turns the active category into Nav2 parameter updates. |
| `metrics_collector` | Accumulates in-run events and hands over a run summary at episode end. |
| `task_manager` | The only node that touches MySQL: reads the task in, writes the run out. |

Nav2 stock nodes (`bt_navigator`, `planner_server`, `controller_server`, `behavior_server`, costmaps, localisation) are **configured, never rebuilt** (PRD NG3).

---

## 5. Standing Constraints

- Python for ROS 2 nodes, C# for Unity. No C++ (NFR-01).
- Exactly one node performs database I/O; no control-loop node queries MySQL at runtime (NFR-03).
- A database outage must never abort or block a simulation run (FR-14, FR-16).
- Diagrams are Mermaid, inline in markdown. No external diagramming tools (PRD NG5).
- ML perception (YOLOv8n) is gated behind the M3/Stage exit gates and never sits on the graded navigation path (FR-21, PRD NG1).

---

## 6. Terms to Avoid

| Don't say | Say instead | Why |
|-----------|-------------|-----|
| detect / classify / recognise an item | look up the item's category | No perception in the graded path (ADR-002). |
| grasp, grip, manipulate, pick up (mechanically) | re-parent (Unity), dwell-then-re-parent | No arm exists (ADR-001). |
| trial, attempt, iteration | episode (live) / run (record) | Keeps the metric vocabulary exact. |
| result, status (of an episode) | outcome | `status` is a `tasks` column. |
| feature (of category profiles) | mechanism reinforcing avoidance | Framing matters for grading (PRD G4). |
| obstacle collision (for namespaces) | namespace conflict | Two different failure modes. |
