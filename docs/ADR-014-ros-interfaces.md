# ADR-014: ROS Interfaces = Standard Messages Only, One `mission` Node

**Status:** Accepted (2026-10-04) · amends [ADR-003](ADR-003-ros1-noetic.md) and [ADR-005](ADR-005-mysql-database.md) · replaces the `warehouse_msgs` package (kept on `Nguyen-planning`)

## Context
Custom `.msg` and `.srv` files must be built with `catkin_make` and regenerated as C# in Unity every time they change. The `Nhan-turtlebot` prototype needs none, because it only uses standard types. The team writes ROS nodes in Python and wants the same on the interface side.

## Decision
1. **No custom message or service types, no `actionlib`, no `dynamic_reconfigure`.** Only standard types. Where a payload has no standard type (acknowledgements, events, results), it is a JSON object in a `std_msgs/String`. The exact types, rates and JSON fields are in [architecture §5](architecture.md#5-interface-contract).
2. **Nodes.**
   - `astar_planner` and `path_follower` in package `cube_control` (from the prototype).
   - One `mission` node in package `warehouse_mission`. It contains `mission_orchestrator`, `metrics_collector`, the motion profile and `task_manager` as classes. MySQL calls run on their own worker thread, so nothing waits on the database ([ADR-005](ADR-005-mysql-database.md)).
   - The headless runner (`warehouse_eval`) is a separate script.
3. **Unity is commanded by one topic per command**, and answers on `/sim/ack`. `mission` waits for the acknowledgement with a timeout.
4. **Profiles are ROS parameters** (`/nav/max_lin`, `/nav/inflation_radius`), read by the planner and the follower.

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
- No catkin build and no C# regeneration when an interface changes. `rostopic list` shows the whole interface.
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
