# ADR-014: ROS Interfaces = Standard Messages Only, One `mission` Node

**Status:** Accepted (2026-10-04) · amends [ADR-003](ADR-003-ros1-noetic.md) and [ADR-005](ADR-005-mysql-database.md) · replaces the `warehouse_msgs` package (kept on `Nguyen-planning`) · updated 2026-10-04 ([ADR-015](ADR-015-physical-waffle-pi-body.md)): topic table, pose on `/robot/pose`, package `turtlebot_control`

## Context
Custom `.msg` and `.srv` files must be built with `catkin_make` and regenerated as C# in Unity every time they change. The `Nhan-turtlebot` prototype needs none, because it only uses standard types. The team writes ROS nodes in Python and wants the same on the interface side.

## Decision
1. **No custom message or service types, no `actionlib`, no `dynamic_reconfigure`.** Only standard types. Where a payload has no standard type (acknowledgements, events, results), it is a JSON object in a `std_msgs/String`. The exact types, rates and JSON fields are in [architecture §5](architecture.md#5-interface-contract).
2. **Nodes.**
   - `astar_planner` and `path_follower` in package `turtlebot_control` (from the prototype's `cube_control`, [ADR-012](ADR-012-custom-python-navigation.md)).
   - One `mission` node in package `warehouse_mission`. It contains `mission_orchestrator`, `metrics_collector`, the motion profile and `task_manager` as classes. MySQL calls run on their own worker thread, so nothing waits on the database ([ADR-005](ADR-005-mysql-database.md)).
   - The headless runner (`warehouse_eval`) is a separate script.
3. **Unity is commanded by one topic per command**, and answers on `/sim/ack`. `mission` waits for the acknowledgement with a timeout. Only `/cmd_vel` exists today; the `/sim/*` commands and `/sim/ack` come with `mission`.
4. **Profiles are ROS parameters** (`/nav/max_lin`, `/nav/inflation_radius`), read by the planner and the follower. Defaults: 0.26 m/s and 0.35 m. The private parameters are `~snap_distance` (planner, 2.0 m, set in `navigation.launch`) and `~waypoint_tol` (follower, 0.15 m).

### Topics in use

These topics exist now. All are standard types. On the ROS side every length and speed is in robot metres, and `map` is the only frame the planner and follower use.

| Topic | Type | Direction and notes |
|-------|------|---------------------|
| `/clock` | `rosgraph_msgs/Clock` | Unity to ROS. Simulation time ([ADR-009](ADR-009-simulation-clock.md)); the nodes run with `use_sim_time` |
| `/robot/pose` | `geometry_msgs/PoseStamped` | Unity to planner and follower. Ground truth, frame `map`, 30 Hz, stamp = simulation time. Unity also publishes the TF `map -> base_footprint` |
| `/map` | `nav_msgs/OccupancyGrid` | Unity to planner. Raw (not inflated), 0.05 m cells, 1 Hz, frame `map`, built from the static `PhysicalBody` colliders at every Play |
| `/cmd_vel` | `geometry_msgs/Twist` | Follower to Unity, in robot metres per second and radians per second. `CmdVelSubscriber` passes it on unchanged and `DiffDriveController` applies the scale ([ADR-010](ADR-010-robot-scale.md)) |
| `/odom` | `nav_msgs/Odometry` | Unity to ROS. Not used by navigation; its TF is off |
| `/scan` | `sensor_msgs/LaserScan` | Unity to ROS. Raycast LIDAR, 5 Hz. Not used by navigation yet |
| `/sim/collision` | `std_msgs/String` (JSON) | Unity to ROS. `{other_tag, sim_time_s, x, y}`, `other_tag` is `Wall`, `Shelf`, `NPC` or `Obstacle`, `x` and `y` in `map`. Debounced to 1 s per object; floor contacts are ignored |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | You or `mission` (or RViz *2D Nav Goal*) to planner. Frame `map` |
| `/planned_path` | `nav_msgs/Path` | Planner to follower. Latched |
| `/nav/cancel` | `std_msgs/Empty` | To follower |
| `/nav/leg_result` | `std_msgs/String` (JSON) | Planner and follower to `mission`: `{"outcome": "succeeded"\|"aborted", "reason": ""\|"no_path"\|"cancelled"}` |

The other topics of the contract in architecture §5 (`/sim/reset_scenario`, `/sim/attach_item`, `/sim/release_item`, `/sim/ack`, `/nav/event`) are planned with `mission` and not built yet.

### What replaces the old contract

| Old (`warehouse_msgs`, `move_base`) | New |
|--------------------------------------|-----|
| `/sim/reset_scenario` service | topic `/sim/reset_scenario` (`std_msgs/String`, scenario id) + `/sim/ack` |
| `/sim/attach_item`, `/sim/release_item` services | topics (`std_msgs/Int32` item id, `std_msgs/Empty`) + `/sim/ack` |
| `/sim/collision` (`CollisionEvent`) | `/sim/collision` (`std_msgs/String`, JSON) |
| `/task_manager/*`, `/metrics/*`, `/motion_profile/set_category` services | calls between classes inside the `mission` node |
| `/move_base` action (`MoveBaseAction`) | `/move_base_simple/goal` + `/nav/cancel` + `/nav/leg_result` |
| `dynamic_reconfigure` on `move_base` | ROS parameters `/nav/max_lin`, `/nav/inflation_radius` |
| `Task`, `RunEvent`, `RunSummary` messages | Python data classes in `mission`; allowed values are listed in architecture §5 |

## Consequences
- No catkin build and no C# regeneration when an interface changes. `rostopic list` shows the whole interface. Unity needs only the standard message classes of the ROS-TCP-Connector; the custom `Assets/RosMessages/Warehouse` of `Nguyen-planning` is not part of this code base.
- JSON in a `String` is not type-checked. The JSON fields are therefore documented in one table and covered by tests in `mission`.
- Acknowledgements and events travel on topics, so a command sent before Unity subscribes is lost. `mission` waits for `/sim/ack` and treats a timeout as a failure of that step.
- One `mission` process means one place to restart and one place for the database thread. If four people need to work in parallel, they split by module (file), not by node.
- A future inventory panel and task queue (manipulator feature) use the same pattern: topics plus acknowledgements, not services.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| Keep `warehouse_msgs` (typed `.msg` / `.srv`) | Rejected by Nguyen: builds and C# regeneration on every change; not what the prototype does |
| Four app nodes talking JSON over `String` topics | Keeps the old split, but every call becomes an untyped message and a timeout case |
| One process for everything, planner included | Simplest, but the planner and follower would restart and fail together with the mission logic |
