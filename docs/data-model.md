# Data Model — UC6 Mobile Robot Warehouse System

Database: MySQL, run via `docker-compose`, seeded by script (ADR-005). Exactly one ROS 2 node, `task_manager`, performs any I/O against this schema at runtime.

## 1. Entities

| Table | Purpose |
|-------|---------|
| `categories` | The three item categories and their motion-profile parameters (fragile/standard/heavy), per `docs/architecture.md` §6. |
| `items` | Item catalog: category, home shelf slot, optional weight. |
| `shelf_slots` | Known static shelf poses (no perception; ADR-002). |
| `tasks` | A pending or completed fulfilment task: item + drop-off pose. |
| `runs` | One row per simulated episode attempt at a task, with aggregate metrics. |
| `run_events` | Fine-grained events within a run: collision, replan, clearance-sample, goal-outcome. |

## 2. Entity-Relationship Summary

- `categories` 1—∞ `items` (an item belongs to exactly one category)
- `shelf_slots` 1—0..1 `items` (a slot may currently hold at most one item)
- `items` 1—∞ `tasks` (an item may be requested by multiple tasks over time)
- `tasks` 1—∞ `runs` (a task may be attempted multiple times, e.g. retries in testing)
- `runs` 1—∞ `run_events` (a run has zero or more discrete events)

See `docs/diagrams.md` §(d) for the Mermaid ER diagram.

## 3. Column-Level Notes

### `categories`
Holds the exact motion-profile parameter set from `docs/architecture.md` §6 so `motion_profile_node` reads them indirectly (via `task_manager`-populated task context, not at runtime — see Non-Functional constraint that no control-loop node queries the database). `max_vel_x`, `acc_lim_x`, `inflation_radius` are stored as authored defaults; runtime application is via ROS parameter services, not live DB reads.

### `items`
`weight_kg` is retained for future refinement (§6 of architecture.md) but is not read by any graded runtime path today.

### `shelf_slots`
`pose_x`, `pose_y`, `pose_theta` are the known, catalog-sourced coordinates that stand in for perception (ADR-002). `occupied_item_id` is nullable and updated when an item is placed/removed in the seed data; it is not updated live during simulation runs (Unity re-parenting is a visual/simulation concern, not a database write).

### `tasks`
`status` moves `pending → in_progress → completed | failed`, written only by `task_manager` at episode start/end.

### `runs`
Carries every metric defined in `docs/prd.md` §8 as a column, so a single row is sufficient for scenario-matrix reporting and CSV export (`docs/test-plan.md`).

### `run_events`
`event_type` is constrained to exactly the four types required: `collision`, `replan`, `clearance_sample`, `goal_outcome`. `payload_json` carries type-specific detail (e.g. clearance value in metres, collision contact point, replan trigger reason, final outcome detail) so the schema does not need one column per event type.

## 4. Indexes and Constraints

| Table | Constraint / Index | Reason |
|-------|----------------------|--------|
| `categories` | `PRIMARY KEY (category_id)`, `UNIQUE (name)` | One row per named category. |
| `items` | `FOREIGN KEY (category_id) REFERENCES categories`, `FOREIGN KEY (home_slot_id) REFERENCES shelf_slots` | Referential integrity to catalog data. |
| `shelf_slots` | `PRIMARY KEY (slot_id)`, `FOREIGN KEY (occupied_item_id) REFERENCES items` (nullable), `INDEX idx_shelf_pose (pose_x, pose_y)` | Fast lookup by approximate location for seed/debug tooling. |
| `tasks` | `FOREIGN KEY (item_id) REFERENCES items`, `INDEX idx_tasks_status (status)` | `task_manager` polls by status at episode start. |
| `runs` | `FOREIGN KEY (task_id) REFERENCES tasks`, `INDEX idx_runs_task (task_id)`, `INDEX idx_runs_scenario (scenario_id)` | Scenario-matrix and per-task queries (`docs/test-plan.md`). |
| `run_events` | `FOREIGN KEY (run_id) REFERENCES runs ON DELETE CASCADE`, `INDEX idx_events_run (run_id)`, `INDEX idx_events_type (event_type)` | Deleting a run (test cleanup) cascades its events; type filtering is used by the CSV exporter. |

## 5. Seed Strategy

1. `docs/schema.sql` is mounted into the MySQL container's `/docker-entrypoint-initdb.d/` via `docker-compose`, so the schema is applied automatically on first container start.
2. A companion seed script (Phase 1 deliverable, not part of this document set) inserts:
   - The three fixed `categories` rows with the parameter values from `docs/architecture.md` §6.
   - A generated `shelf_slots` grid (e.g. aisle × bay coordinates) sized to the Unity warehouse scene.
   - A sample `items` catalog referencing those slots and categories.
   - An initial `tasks` queue (`status = 'pending'`) covering the scenario matrix in `docs/test-plan.md`.
3. Re-seeding is idempotent: the seed script truncates `tasks`, `runs`, `run_events` (test data) but preserves `categories`, `items`, `shelf_slots` (catalog data) unless a `--full-reset` flag is passed.

## 6. Telemetry Write Path

1. At episode start, `task_manager` calls `SELECT ... FROM tasks WHERE status='pending' ORDER BY priority, created_at LIMIT 1 FOR UPDATE`, marks it `in_progress`, and returns it to `mission_orchestrator` via the `/task_manager/get_next_task` service (`docs/architecture.md` §3).
2. Throughout the run, `metrics_collector` accumulates events in memory (no DB writes during the run — NFR-03).
3. At episode end, `mission_orchestrator` publishes the final outcome; `metrics_collector` finalises its summary and hands it to `task_manager` on `/task/next_task_result`.
4. `task_manager` performs a single transaction:
   ```sql
   START TRANSACTION;
   UPDATE tasks SET status = <completed|failed> WHERE task_id = ?;
   INSERT INTO runs (...) VALUES (...);
   INSERT INTO run_events (...) VALUES (...), (...), ...;
   COMMIT;
   ```
5. If the transaction fails (MySQL unreachable), the full payload is serialised to a local disk buffer file and retried at the next episode start before that episode's own read (FR-16); the simulation itself has already ended by this point, so this failure mode cannot block a run in progress.
