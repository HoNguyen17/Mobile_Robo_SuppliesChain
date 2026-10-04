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
| **Static map** | Walls + shelves, built by Unity at Play and sent raw (not inflated). Scenario obstacles are **not** in it. |
| **Static obstacle** | A non-moving obstacle not in the map (boxes). Tested in M2. |
| **NPC** | A Unity-scripted moving person. Not a ROS node. Tested in M3. |
| **Inflation** | The planner keeps the robot centre at least `inflation_radius` away from every obstacle. A hard limit, set by the motion profile. |
| **Global plan** | A* path over the whole map (`astar_planner`). |
| **Obstacle layer** | Cells marked blocked from what the emulated LIDAR sees, on top of the static map. |
| **Follower** | `path_follower`: drives along the global plan and stops when the way ahead is blocked. |
| **Replan** | A new global plan made because the robot was blocked. |
| **Recovery** | The follower stops because the path ahead is blocked and the planner replans; if no path exists, the robot rotates in place and retries. One stop + replan is one recovery. |
| **Clearance** | Closest LIDAR distance minus the robot radius. |
| **Collision** | A Unity contact between the robot and a wall, shelf, obstacle or NPC. What the grade punishes. |

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
| **Robot metres** | Unity units divided by the robot scale (3.2). Every length, speed and range on the ROS side uses them. |
| **Kinematic body** | The robot is the Unity object `Cube`, moved by its transform. No physics, no wheel dynamics. A Waffle Pi model is attached for looks only. |

## Words to avoid

| Don't say | Say | Why |
|-----------|-----|-----|
| detect / classify an item | look up the item's category | No perception |
| grasp / pick up (mechanically) | attach | No arm |
| trial / attempt / iteration | episode (live) · run (record) | Keeps metrics exact |
| result / status of an episode | outcome | `status` belongs to `tasks` |
| Nav2, Stage A/B/C, namespace | — | Removed: we use ROS 1 and one robot |
| move_base, AMCL, costmap, DWA, TEB | planner, follower, obstacle layer | Not used: we run our own Python navigation ([ADR-012](docs/ADR-012-custom-python-navigation.md)) |
