# ADR-012: Navigation = Custom Python A* Planner + Path Follower

**Status:** Accepted (2026-10-04) · supersedes [ADR-008](_archive/2026-10-04-move-base-plan/ADR-008-navigation-stack.md) and [ADR-004](_archive/2026-10-04-move-base-plan/ADR-004-local-planner.md) · amended 2026-10-04 ([ADR-015](ADR-015-physical-waffle-pi-body.md)): package `turtlebot_control`, pose on `/robot/pose`, physical Waffle Pi

## Context
The team's code base is the `Nhan-turtlebot` prototype: a Python A* planner and a path follower (`cube_control`) that drove a kinematic robot in Unity. ADR-008 had chosen `move_base` + AMCL and had rejected "write our own planner node in Python". That choice is reversed.

The algorithm and the structure of the prototype are kept in the package **`ros1/turtlebot_control`**, which drives the physical Waffle Pi ([ADR-015](ADR-015-physical-waffle-pi-body.md)). `cube_control` is the earlier prototype. It was archived after the integrated Unity + ROS run passed (2026-10-05), in [`_archive/ros1-cube-prototype/`](../_archive/ros1-cube-prototype/README.md).

The course accepts a custom planner for M1 to M3. Nguyen confirmed this on 2026-10-04; add the written source here when it is available.

## Decision

| Piece | Choice |
|-------|--------|
| Package | `ros1/turtlebot_control`: nodes `astar_planner.py` and `path_follower.py`; `grid_planner.py`, `path_tracker.py` and `nav_common.py` are plain Python without `rospy`, so they are unit tested anywhere. `warehouse_bringup` starts them with the bridge and RViz |
| Pose | Ground truth from Unity on `/robot/pose` (`PoseStamped`, frame `map`, 30 Hz, simulation time). No AMCL, and navigation does not use `/odom`. Unity also publishes the TF `map -> base_footprint` for RViz. The only frame the nodes use is `map` |
| Map | Raw static map built by Unity at every Play and sent on `/map` at 1 Hz. A cell is blocked when a non-trigger collider of a **static** `PhysicalBody` (walls, stations, racks; [ADR-011](ADR-011-physics-standard.md)) reaches into the band 0.03 to 1.0 m above the robot base. Cells are 0.05 m. The grid is **not** inflated, and dynamic boxes are not in it |
| Inflation | Done in the planner: the robot centre stays at least `inflation_radius` away from every obstacle. Once profiles exist the value comes from the category profile; until then the default is **0.35 m** (`navigation.launch`; the per-category values in [architecture §9](architecture.md#9-category-motion-profiles) are older and will be re-tuned). The Waffle Pi footprint reaches **0.257 m** from the wheel axis centre. The planner keeps one inflated copy of the map per radius |
| Obstacles not in the map | Unity emulates the LIDAR with raycasts (`/scan`, [ADR-015](ADR-015-physical-waffle-pi-body.md)). **The planner does not use it yet.** Planned: `astar_planner` marks the cells that the beams hit on top of the map (obstacle layer) |
| Global planner | `astar_planner`: 8-connected A* (octile heuristic, no corner cutting), goal snapping, line-of-sight smoothing. Publishes `/planned_path`. A start or goal in a blocked cell moves to the nearest free cell within `~snap_distance` (2.0 m; a goal only to a cell the robot can reach). It rejects a goal that is not in frame `map` or is NaN, a missing map or pose, and a pose older than 1 s, and then reports `aborted` / `no_path` |
| Follower | `path_follower`: waypoint follower with a P controller at 20 Hz. Its speed limit is `max_lin` from the profile. Heading gain 2.0; it turns on the spot when the heading error is above 45 degrees; angular speed at most 1.0 rad/s (the Waffle Pi can do 1.82); a corner counts as reached within `~waypoint_tol` (0.15 m) and the goal within 0.10 m; it slows down towards the last waypoint. It stops when `/robot/pose` is older than 0.5 s of simulation time |
| Recovery | **Planned, not built** (outside the current change). Today a leg with no path ends `aborted` / `no_path`. The plan: the follower stops when `/scan` shows the path ahead blocked, and the planner replans. If no path exists, the robot rotates in place and tries again. Each stop + replan is one **recovery** (`/nav/event`). After repeated failures the leg ends as `aborted` |
| Profile | ROS parameters `/nav/max_lin` (default 0.26 m/s, read at every step) and `/nav/inflation_radius` (default 0.35 m, read at every plan), in robot metres, set by `mission` before each leg ([architecture §9](architecture.md#9-category-motion-profiles)). Until `mission` exists, `navigation.launch` sets the defaults |
| Leg result | One JSON on `/nav/leg_result` per leg: `succeeded`, or `aborted` with reason `no_path` (planner) or `cancelled` (`/nav/cancel`, or Unity restarted its clock) |
| Time | Simulation time ([ADR-009](ADR-009-simulation-clock.md)). The follower waits for the first `/clock`, and a Unity Play restart ends the running leg as `aborted` / `cancelled` while both nodes keep running |
| Units | Robot metres in every ROS-side number. Unity scale 4 never reaches ROS ([ADR-010](ADR-010-robot-scale.md)) |
| Interface | Standard messages only ([ADR-014](ADR-014-ros-interfaces.md)) |

## Consequences
- The stock `navigation` stack is not used: no `move_base`, `amcl`, `map_server`, `gmapping`, costmaps, `dwa_local_planner` or `teb_local_planner`. NFR-01 still holds, because our nodes are Python.
- **Clearance risk.** Inflation here is a hard limit, not a cost like in `move_base`. A path that hugs an obstacle keeps only `inflation_radius - 0.257 m` (the footprint radius) of clearance, which is about 0.09 m with the default 0.35 m. S-01 (>= 0.15 m) and S-02 (>= 0.10 m) may need a proximity cost in A* or the one allowed threshold revision ([test-plan.md](test-plan.md)). **Open:** decide after the first S-01 dry run.
- S-04 relies on stop + replan. There is no velocity-based local planner. If S-04 fails its gate, add a slow/stop zone around NPCs before considering another planner.
- Speed is not a concern: about 40 ms per plan on a 0.05 m map.
- A physical body can slide and tip, and collisions are real contacts: static shelves and walls stop the robot, and `CollisionReporter` records them on `/sim/collision` for the metrics. The inflated map is the only protection against a first contact. A dynamic box in an aisle is not in the map until the scan-based layer exists.
- Gaps found in the prototype are fixed in `turtlebot_control` (tests in `ros1/turtlebot_control/test`): the goal-snap area is a circle; line of sight walks the exact cells of the segment; `path_follower` swaps its path under a lock and stops when `/robot/pose` is older than 0.5 s; `/nav/cancel`, `/nav/leg_result` and `/nav/inflation_radius` exist; both nodes read `/robot/pose` as `PoseStamped`. The prototype's map missed the `WallPanel` objects; the Unity map now comes from the static colliders, so that gap is closed.
- **Verified so far.** 109 unit tests pass on Windows (Python 3.14) and in WSL (Python 3.8.10, the Noetic version). They include a closed-loop model of the Waffle Pi (limits 0.26 m/s and 1.82 rad/s, ramps 1.0 m/s^2 and 3.0 rad/s^2, wheel slip, 30 Hz pose with latency) that drives the real follower logic and checks that the footprint circle (0.257 m) never touches a wall. In WSL, the real nodes built with `catkin_make` also ran against a fake Unity (simulation time, clock restart): around a wall, cancel, a closed pocket (`no_path`), `/nav/max_lin`, `/nav/inflation_radius`, a stale pose, bad goals, a clock restart, and a follower restart that ignores the latched path.
- **Verified in Unity (reported by the user, 2026-10-05, no figures).** The Edit Mode and Play Mode tests are green in the Unity Editor, and the integrated Unity + ROS acceptance run passed ([ADR-015](ADR-015-physical-waffle-pi-body.md), [milestones.md](milestones.md)).
- **Not verified yet.** The scan-based obstacle layer, recovery and the S-xx scenarios.
- **Fallback.** Branch `Nguyen-planning` keeps the full `move_base` plan; its physical body is now part of this code base too. If the custom planner cannot pass M2 or M3, that is the way back.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| Keep `move_base` + AMCL (ADR-008) | The team code base is the Python planner. The `move_base` plan stays on `Nguyen-planning` as the fallback |
| A* inside Unity (`TurtleBotNavigator`, earlier `CubeCarNavigator`) | Planning must run in ROS. `TurtleBotNavigator` stays as a Unity-only test mode, and Unity keeps only the map builder |
| Python planner + AMCL | Needs tuning in a repetitive warehouse. Localisation is not graded |
