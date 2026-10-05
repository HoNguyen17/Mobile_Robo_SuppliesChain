# CONTEXT: Glossary

Use these words, with these meanings, in code, issues, commits and tests. Start with [docs/README.md](docs/README.md) for the project overview.

---

## The big picture in one sentence

**Unity is the body, ROS is the brain, MySQL is the notebook.** The project exists to prove **obstacle avoidance**. Everything else is support.

## Two axes: don't mix them up

| Axis | Values | Means |
|------|--------|-------|
| **Milestone** | M1, M2, M3 | What the course grades: navigate · static avoidance · dynamic avoidance |
| **Phase** | P0–P4 | Where the team is in the 8-week calendar ([milestones.md](docs/milestones.md)) |

## Words

### Work and records
| Term | Meaning |
|------|---------|
| **Task** | Requested work: one item to move to one drop-off pose. A row in `tasks`. |
| **Episode** | One live attempt at a task, from dispatch to report. The runtime word. |
| **Run** | The stored record of one episode (`runs` row + `run_events`). The data word. |
| **Outcome** | `success` or `fail` of an episode. If `fail`, there is also a **fail reason**. |
| **Leg** | One planner goal. Every episode has two: the **shelf leg** and the **drop-off leg**. |
| **Leg result** | How a leg ended, as JSON on `/nav/leg_result`: `succeeded`, or `aborted` with the reason `no_path` or `cancelled`. Sent by the planner and the follower to the mission. It is not the episode **outcome**. |

### Items
| Term | Meaning |
|------|---------|
| **Item** | A catalog entry with a category and a shelf slot. Known from the database, never detected. |
| **Category** | Fragile, Standard or Heavy. It only selects a motion profile. |
| **Motion profile** | The speed limit (`max_lin`) and inflation radius (`inflation_radius`) for a category. Applied before the drop-off leg. |
| **Shelf slot** | A known pose where an item lives. |
| **Dwell** | A deliberate pause on arrival, before attach or release. |
| **Attach / Release** | Unity re-parents the item onto the robot, or back off it. No grasping; there is no arm. |

### Navigation
| Term | Meaning |
|------|---------|
| **Static map** | Walls + shelves: the static bodies of the warehouse. Unity builds it at every Play and sends it as the **raw map**. Scenario obstacles and dynamic boxes are **not** in it. |
| **Raw map** | The occupancy grid on `/map`: 0.05 m cells, frame `map`, 1 Hz, not inflated. Built from the static `PhysicalBody` colliders in a height band of 0.03 to 1.0 m above the robot base. Rebuilt from the colliders at every Play. |
| **Inflated map** | The planner's own copy of the raw map, with every obstacle grown by `inflation_radius`, cached per radius. The raw map is never changed. |
| **Static obstacle** | A non-moving obstacle not in the map (boxes). Tested in M2. |
| **NPC** | A Unity-scripted moving person. Not a ROS node. Tested in M3. |
| **Inflation** | The planner keeps the robot centre at least `inflation_radius` away from every obstacle. A hard limit, set by the motion profile. Default 0.35 m, which is about 9 cm more than the **footprint**. |
| **Global plan** | A* path over the whole map (`astar_planner`). |
| **Obstacle layer** | Cells marked blocked from what the emulated LIDAR sees, on top of the static map. |
| **Follower** | `path_follower`: drives along the global plan and stops when the way ahead is blocked. |
| **Replan** | A new global plan made because the robot was blocked. |
| **Recovery** | The follower stops because the path ahead is blocked and the planner replans; if no path exists, the robot rotates in place and retries. One stop + replan is one recovery. |
| **Clearance** | Closest LIDAR distance minus the footprint radius (0.257 m). |
| **Collision** | A Unity contact between the robot and a wall, shelf, obstacle or NPC. What the grade punishes. `CollisionReporter` publishes it on `/sim/collision`; floor contacts are ignored. |

### Verification
| Term | Meaning |
|------|---------|
| **Scenario** | A test setup, S-00 … S-05, with numeric pass/fail criteria ([test-plan.md](docs/test-plan.md)). |
| **Gate** | The exit conditions of a phase. Pass/fail, never waived silently. |
| **Runner** | The script that runs a scenario 20 times and writes the CSVs. |

### Integration
| Term | Meaning |
|------|---------|
| **Bridge** | Unity ROS-TCP-Connector ↔ `ros_tcp_endpoint`, over `127.0.0.1:10000`. |
| **Sim time** | The time published by Unity on `/clock`. All ROS nodes use it. |
| **Robot metres** | Unity units divided by the robot scale (4): 1 robot metre = 4 Unity units ([ADR-010](docs/ADR-010-robot-scale.md)). Every length, speed and range on the ROS side uses them. ROS `x` = Unity `Z` / 4, ROS `y` = −Unity `X` / 4. |
| **Physical body** | The robot is the TurtleBot3 Waffle Pi model with `ArticulationBody` wheels, driven by `DiffDriveController` from `/cmd_vel` ([ADR-015](docs/ADR-015-physical-waffle-pi-body.md)). It can slide or tip, and static shelves and walls stop it. Not the `PhysicalBody` component, which tags warehouse objects for the physics standard. |
| **Physics standard** | The one set of physical rules for the Unity scene: gravity 39.24 m/s² (4 × 9.81), real densities and friction, a `PhysicalBody` on the shell, stations, racks (static) and boxes (dynamic). Applied by *Robotics > Warehouse > Apply Physics Standard* ([ADR-011](docs/ADR-011-physics-standard.md)). |
| **Ground-truth pose** | The true robot pose on `/robot/pose` (`PoseStamped`, frame `map`, 30 Hz, stamp = sim time), and the TF `map` → `base_footprint`. Read from the simulated body, not estimated. The planner and the follower use it, not `/odom`. |
| **Footprint** | The circle around the wheel-axis centre that holds the Waffle Pi: radius 0.257 m. Used for clearance and for the closed-loop tests. |

## Words to avoid

| Don't say | Say | Why |
|-----------|-----|-----|
| detect / classify an item | look up the item's category | No perception |
| grasp / pick up (mechanically) | attach | No arm |
| trial / attempt / iteration | episode (live) · run (record) | Keeps metrics exact |
| result / status of an episode | outcome | `status` belongs to `tasks` |
| Nav2, Stage A/B/C, namespace | — | Removed: we use ROS 1 and one robot |
| move_base, AMCL, costmap, DWA, TEB | planner, follower, obstacle layer | Not used: we run our own Python navigation ([ADR-012](docs/ADR-012-custom-python-navigation.md)) |
| Cube, kinematic body | physical body, the robot | The kinematic `Cube` was dropped on 2026-10-04 ([ADR-015](docs/ADR-015-physical-waffle-pi-body.md)) |
