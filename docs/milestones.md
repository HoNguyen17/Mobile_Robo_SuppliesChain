# Milestones — UC6 Mobile Robot Warehouse System

8-week timeline, 4-person team. Phase 0 completes in Week 1 per the course brief.

## 1. Owner Roles (4-person team)

| Role | Primary responsibility |
|------|--------------------------|
| Nav2/Planning Lead | Global/local planner configuration, recovery behaviours, ADR-004 benchmark. |
| Unity/Bridge Lead | Unity scene, robot controller, ROS-TCP bridge, pickup/place/collision event contract, NPC scripting. |
| Backend/Data Lead | MySQL schema/seed, `task_manager`, telemetry write path, docker-compose. |
| Integration/Test Lead | `mission_orchestrator`, `metrics_collector`, headless runner, CSV export, demo script. |

All four collaborate on multi-robot staging (Stage B/C) once Stage A's exit gate is met, since it touches every area.

## 2. Phase Plan

| Phase | Weeks | Focus | Entry gate | Exit gate |
|-------|:--:|-------|-------------|-----------|
| Phase 0 | 1 | Environment verification | Team formed, repo/tool access confirmed | Tutorial repo runs unmodified; ROS 2 distro pinned (ADR-003 finalised); Unity version confirmed; `docker-compose up` succeeds with `docs/schema.sql` applied; TurtleBot3 Waffle Pi model/plugin availability confirmed |
| Phase 1 | 2–3 | M1: tutorial adaptation to warehouse scenario | Phase 0 exit gate met | M1 acceptance criteria met (`docs/test-plan.md` S-00): goal success rate ≥ 90% over N=20, 0 collisions; `task_manager`/`mission_orchestrator` skeleton operational against seeded MySQL |
| Phase 2 | 3–4 | M2: static obstacle avoidance | Phase 1 exit gate met | M2 acceptance criteria met (S-01–S-03); category profile mechanism wired end-to-end (even with only the Standard profile exercised) |
| Phase 3 | 5–6 | M3: dynamic obstacle avoidance | Phase 2 exit gate met | M3 acceptance criteria met (S-04); ADR-004 benchmark completed and local planner finalised; recovery/replanning verified; per-category velocity difference proven (S-06) |
| Phase 4 | 6–7 | Multi-robot Stage B, then Stage C if time permits | Phase 3 exit gate met (Stage A baseline) | Stage B gate: 2 robots run without TF/namespace conflict over N=20 in S-05; if met, attempt Stage C gate (3 robots) within remaining Week 7 time; if either gate fails, ship the prior stage (ADR-006) |
| Phase 5 | 8 | Hardening, evidence, demo | Phase 3 exit gate met at minimum (Stage A is sufficient) | Headless runner has produced N≥20 CSV-exported runs per required scenario; manual demo sequence rehearsed at least twice; documentation finalised; submission packaged |

## 3. Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|:--:|:--:|------------|
| Tutorial repo API drift or ROS 2 distro incompatibility | Medium | High | Phase 0 verification task (ADR-003); pin an exact commit/tag, not just a branch name |
| ROS-TCP-Endpoint bandwidth/connection-count limits under multi-robot | Medium–High | Medium | Stage gating (ADR-006); per-robot endpoint fallback documented in `docs/architecture.md` §7; load-test before Stage B |
| DWB vs TEB benchmark inconclusive or overruns | Medium | Medium | Time-boxed to 2 days (ADR-004); default to DWB if inconclusive |
| MySQL/docker-compose environment issues on student machines | Medium | Low | DB failure fallback already architected (FR-16); Phase 0 includes a `docker-compose up` dry run on every team member's machine |
| Category profile parameters cause instability (e.g. Fragile too slow, causing timeouts) | Medium | Medium | Values documented as tunable defaults with stated basis (`docs/architecture.md` §6); revisit after Phase 2/3 dry runs |
| Scope creep toward ML perception (YOLOv8n) before graded milestones are secure | Medium | High (grading risk) | Explicit non-goal (PRD NG1); FR-21 gates any YOLOv8n work behind the M3/Stage exit gates |
| Multi-robot TF/namespace conflicts | Medium | Medium | Namespacing/TF-prefix scheme fixed once in `docs/architecture.md` §7; Stage B dry-run required before Stage C is attempted |
| Grading-day demo failure (flaky simulation) | Low–Medium | High | Scripted manual demo sequence rehearsed in Phase 5 (`docs/test-plan.md`); headless automated CSV evidence available as backup if the live demo falters |
| Team member unavailability during a critical week | Low–Medium | Medium | Owner roles documented above so any two members can cover a phase's exit-gate work if one is unavailable |
