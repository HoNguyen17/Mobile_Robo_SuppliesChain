# ADR-005: Database — MySQL via docker-compose, Single Writer Node

## Context
The system needs to persist a task catalog and per-run telemetry for grading evidence, without that persistence ever being able to interfere with real-time navigation behaviour or block a simulation run.

## Decision
Use MySQL, run via `docker-compose`, with the schema in `docs/schema.sql` applied on container init and seeded by a script (`docs/data-model.md` §5). Exactly one ROS 2 node, `task_manager`, performs any database I/O: it reads the next task at episode start and writes `runs`/`run_events` at episode end, in a single transaction. No planner, controller, or other control-loop node queries MySQL at runtime (NFR-03). A database outage at either point falls back to a local, non-blocking path (FR-16) and never aborts or delays the simulation itself.

## Consequences
- Database access is trivially easy to reason about and test in isolation, since it is confined to one node with two call sites (episode start, episode end).
- The single-writer pattern means telemetry write throughput is bounded by one connection; this is not a concern at the project's scale (tens of runs, not a production workload).
- The fallback path (local disk buffering, retry-on-next-start) must itself be tested, since "the database must never block a run" is only true if that fallback actually works.

## Alternatives Rejected
- **PostgreSQL:** a reasonable alternative with equivalent transactional guarantees; not selected because MySQL is the course-locked choice for this project.
- **SQLite (file-based, no server):** rejected — `docker-compose`-managed MySQL was specified, and a shared server better resembles a realistic WMS backend for the scenario framing.
- **Direct database access from `mission_orchestrator` or Nav2 nodes:** rejected outright — this would put database latency/availability on the real-time control path, exactly what the single-writer constraint is designed to prevent.
