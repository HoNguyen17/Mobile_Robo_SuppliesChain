# ADR-005: MySQL, with a Single Database Node

**Status:** Accepted · amended 2026-10-04 ([ADR-014](ADR-014-ros-interfaces.md)): `task_manager` is now a class inside the `mission` node

## Context
We need to store tasks and results for grading, but the database must never slow down or block navigation.

## Decision
- Use **MySQL 8** in Docker (Docker Desktop, used from WSL via WSL integration). The schema is loaded on first start ([schema.sql](schema.sql)).
- **Only the `task_manager` class** (inside the `mission` node) touches the database. It runs on its own worker thread, and only at two moments: episode start (read a task) and episode end (write the run, in one transaction).
- If the DB is down: at start, use a bundled default task; at end, write JSON to `~/.warehouse/pending/` and upload it next time.

## Consequences
- Easy to test: all database code sits in one class with two calls.
- The fallback path must itself be tested (stop the container mid-batch).
- Passwords live in `infra/.env`, which is git-ignored.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| PostgreSQL | Equivalent, but MySQL is the course choice |
| SQLite | Simpler, but MySQL is required |
| DB access from navigation nodes | Would put DB latency on the control loop |
