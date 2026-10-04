# ADR-012: Navigation = Custom Python A* Planner + Path Follower

**Status:** Accepted (2026-10-04) · supersedes [ADR-008](_archive/2026-10-04-move-base-plan/ADR-008-navigation-stack.md) and [ADR-004](_archive/2026-10-04-move-base-plan/ADR-004-local-planner.md)

## Context
The team's code base is now the `Nhan-turtlebot` prototype: a Python A* planner and a path follower (`ros1/cube_control`) that drive a kinematic robot in Unity. ADR-008 had chosen `move_base` + AMCL and had rejected "write our own planner node in Python". That choice is reversed.

The course accepts a custom planner for M1 to M3. Nguyen confirmed this on 2026-10-04; add the written source here when it is available.

## Decision

| Piece | Choice |
|-------|--------|
| Pose | Ground truth from Unity on `/cube/pose` (`PoseStamped`). No AMCL, no `/odom`, no TF tree. The only frame is `map` |
| Map | Raw static map (walls and shelves) built by Unity at Play and sent on `/map`. Obstacles are the objects tagged `Shelf` and `WallPanel`. The grid is **not** inflated |
| Inflation | Done in the planner: the robot centre stays at least `inflation_radius` away from every obstacle. The value comes from the category profile |
| Obstacles not in the map | Unity emulates the LIDAR with raycasts (`/scan`, [ADR-013](ADR-013-kinematic-robot-body.md)). `astar_planner` marks the cells that the beams hit on top of the map (obstacle layer) |
| Global planner | `astar_planner`: 8-connected A* (octile heuristic), goal snapping, line-of-sight smoothing. Publishes `/planned_path` |
| Follower | `path_follower`: waypoint follower with a P controller. Its speed limit is `max_lin` from the profile |
| Recovery | The follower stops when `/scan` shows the path ahead blocked, and the planner replans. If no path exists, the robot rotates in place and tries again. Each stop + replan is one **recovery** (`/nav/event`). After repeated failures the leg ends as `aborted` |
| Profile | ROS parameters `/nav/max_lin` and `/nav/inflation_radius`, in robot metres, set by `mission` before each leg ([architecture §9](architecture.md#9-category-motion-profiles)) |
| Interface | Standard messages only ([ADR-014](ADR-014-ros-interfaces.md)) |

## Consequences
- The stock `navigation` stack is not used: no `move_base`, `amcl`, `map_server`, `gmapping`, costmaps, `dwa_local_planner` or `teb_local_planner`. NFR-01 still holds, because our nodes are Python.
- **Clearance risk.** Inflation here is a hard limit, not a cost like in `move_base`. A path that hugs an obstacle keeps only `inflation_radius - 0.21 m` (the footprint radius) of clearance, which is 0.09 m for Standard. S-01 (>= 0.15 m) and S-02 (>= 0.10 m) may need a proximity cost in A* or the one allowed threshold revision ([test-plan.md](test-plan.md)). Decide after the first S-01 dry run.
- S-04 relies on stop + replan. There is no velocity-based local planner. If S-04 fails its gate, add a slow/stop zone around NPCs before considering another planner.
- Speed is not a concern. The planner logic alone took 5 ms on a 77 x 70 grid and 0.7 s on a 532 000-cell grid (measured 2026-10-04 with `rospy` stubbed and sparse obstacles).
- Known gaps in the prototype, to fix in P1 ([milestones.md](milestones.md)): the goal-snap window is a square instead of a radius; line-of-sight samples can miss a blocked corner; `path_follower` can read a half-updated path and does not check that the pose is fresh; there is no leg result for the mission; the map misses the `WallPanel` objects.
- **Fallback.** Branch `Nguyen-planning` keeps the full `move_base` plan and the physical body. If the custom planner cannot pass M2 or M3, that is the way back.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| Keep `move_base` + AMCL (ADR-008) | The team code base is the Python planner. The stock stack also needs `/odom`, TF and a physical body, which this code base does not have |
| A* inside Unity (`CubeCarNavigator`) | Planning must run in ROS. Unity keeps only the map builder |
| Python planner + AMCL | Needs `/odom`, TF and tuning in a repetitive warehouse. Localisation is not graded |
