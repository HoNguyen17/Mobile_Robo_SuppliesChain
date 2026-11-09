# ADR-017: The Live Costmap Shows the Radius of the Plan Being Followed

**Status:** Accepted (2026-11-09) · applies to `astar_planner` and `dstar_planner` · extends [ADR-012](ADR-012-custom-python-navigation.md), follows [ADR-016](ADR-016-planner-scheduling.md)

## Context
The planners publish `/costmap` (latched) from two places, with two different inflation radii (open issue 11):
- After every plan, the costmap of the radius the plan was found with. The planner tries 0.40 m, the nominal `/nav/inflation_radius` (0.35 m), 0.30 m and 0.28 m and keeps the first that gives a path, so usually 0.40 m.
- While a leg runs, `on_scan` rebuilds the costmap every 0.5 s from the lidar hits. It used the nominal 0.35 m.

RViz therefore alternated between a 0.40 m and a 0.35 m picture of the same obstacles, and the picture of the live costmap did not match what the planner and the early-replan check (`plan radius − 0.02`) were using.

## Decision
- A pure function `costmap_radius(plan_radius, nominal)` in `nav_common.py` returns the radius of the current plan (`current_path_radius`) while there is one, and the nominal radius otherwise. A non-finite, non-positive or non-numeric plan radius falls back to the nominal one.
- Both planners' `publish_live_costmap` use it. After `/nav/cancel` ([ADR-016](ADR-016-planner-scheduling.md)) `current_path_radius` is cleared, so the live costmap returns to the nominal radius.
- Planning itself is unchanged: same candidate radii, same order.

## Consequences
- `/costmap` shows one radius for the whole leg, the one the planner used and the one the early-replan check is based on.
- D\* Lite builds its incremental live inflation once per radius, so one rebuild happens right after a plan when the chosen radius differs from the nominal one; it is then reused. It was already rebuilt whenever the radius changed.
- The live costmap is only for display and for `ab_run`'s logs; no node uses it for control.
- Tests: `CostmapRadiusTest` in `test_nav_common.py` (the function), a source check in `test_planner_scheduling_source.py`, and `test_planner_nodes.py` (both real nodes: nominal before a plan, the plan radius during a leg, nominal again after a cancel). Checking it by eye in RViz is a manual step for the milestone 3 setup.
