# ROS1 + Unity prototype: warehouse robot (TurtleBot3 model) with A* navigation

A robot drives through the Unity warehouse scene (`WarehouseProjectURP`). Unity and ROS1 talk over ROS-TCP-Endpoint. ROS plans a path with A* on an occupancy grid published by Unity, and a follower node sends velocity commands back. The robot is shown as a **TurtleBot3 Waffle Pi** model.

The course requires **ROS1** (not ROS 2).

## Scope of this prototype (read this first)

The controlled body is the Unity object named **`Cube`**. A script (`CubeCmdVelSubscriber`) moves its transform according to `/cmd_vel`. The TurtleBot3 is a **kinematic visual model**: a rigid child of the Cube, scaled up for visibility.

- Simulated: the ROS interface (`/cmd_vel` in, `/cube/pose` out), the occupancy grid, A* planning, waypoint following, goal arrival.
- **Not simulated:** physics, wheel dynamics (the wheels do not turn), collisions, and sensors (no LiDAR, no odometry). The robot avoids shelves only because the planner inflates obstacles on the map.

This is a deliberate first milestone (static obstacle avoidance). Physical wheel control and sensors are listed under "Next steps".

## Environment

| Part | Version / setup |
|---|---|
| Unity | 2021.1.11f1, project `WarehouseProjectURP` |
| OS | Windows + WSL2, distro **Ubuntu-20.04** (check with `lsb_release -a`) |
| ROS | ROS1 Noetic |
| Bridge | Unity ROS-TCP-Connector (Unity side) + ROS-TCP-Endpoint (ROS side) |

## Repository layout

```
WarehouseProjectURP/        Unity project (Assets, Packages, ProjectSettings)
  Assets/URDF/              TurtleBot3 Waffle Pi (URDF, meshes, prefab, materials)
  Assets/Editor/            StripPhysicsFromRobot.cs (editor menu: Tools > Robot > Strip Physics (Visual Only))
ros1/cube_control/          ROS1 package (Python nodes in scripts/)
com.unity.robotics.warehouse.*/   local packages the Unity project depends on (do not delete)
README_ROS1_Prototype.md    this file
```

The other folders in the repo come from the original Unity warehouse project. `WarehouseProjectURP/Packages/manifest.json` references the local `com.unity.robotics.warehouse.*` packages, so they must stay.

## Setup

### 1. Unity

1. Install Git (Unity downloads `ros-tcp-connector` and `urdf-importer` from GitHub on first open).
2. Open `WarehouseProjectURP` with Unity 2021.1.11f1 and wait for the packages to resolve.
3. In **Robotics > ROS Settings**: protocol **ROS1**, IP `127.0.0.1`, port `10000`.
4. On the `Cube` object, tick these scripts: `CubePosePublisher`, `CubeCmdVelSubscriber`, `OccupancyGridPublisher`.
5. Leave `CubeCarNavigator` **unticked** while ROS drives the robot. It is the original Unity-side planner and would fight the ROS commands.
6. The scene already contains `turtlebot3_waffle_pi` as a child of `Cube` (see the next section). Nothing to do unless you rebuild it.

### 2. ROS (inside WSL Ubuntu-20.04)

```bash
cd ~/catkin_ws/src
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git   # ROS1 version (main branch)
cp -r /path/to/repo/ros1/cube_control .
cd ~/catkin_ws
catkin_make
source devel/setup.bash
chmod +x src/cube_control/scripts/*.py
```

Add `source ~/catkin_ws/devel/setup.bash` to `~/.bashrc` so every terminal has the workspace.

Python files must keep Unix (LF) line endings. The repo's `.gitattributes` enforces this. If `rosrun` fails with `python3\r: No such file or directory`, the file has Windows line endings.

## TurtleBot3 visual model

**How it is attached.** `turtlebot3_waffle_pi` is a child of `Cube`:

| Object | Setting |
|---|---|
| `Cube` | Mesh Renderer **unticked** (hidden), Box Collider 1 x 1 x 1 and the ROS scripts kept, scale (1, 1, 1), rotation 0 |
| `turtlebot3_waffle_pi` | local position (0, -0.5, 0) so the wheels touch the floor, **scale (3.2, 3.2, 3.2)**, physics components removed |

**Why 3.2x.** The real Waffle Pi is about 0.281 x 0.306 m, which is tiny next to the 1 m Cube and hard to see. At 3.2x it is about 0.98 m wide and fits inside the Cube's 1 m footprint, which is what the planner assumes (**Robot Radius 1**, unchanged). The follower speed limit also stays at the cube values (`MAX_LIN = 0.5`). It is therefore a scaled-up TurtleBot, not a true-size one.

**How it was imported** (only needed if you rebuild it):

1. In WSL: `sudo apt install ros-noetic-turtlebot3-description`, then convert `turtlebot3_waffle_pi.urdf.xacro` to a plain `.urdf` with `xacro`.
2. Copy the `.urdf` and the 4 STL meshes (`waffle_pi_base`, `lds`, `left_tire`, `right_tire`) into Unity. The importer appends the `package://` path to the folder containing the URDF, so the `.urdf` must sit **directly in `Assets/URDF/`**, next to the `turtlebot3_description/` folder, not in a `urdf/` subfolder.
3. Import with **Y Axis** and **VHACD** collision.
4. Run **Tools > Robot > Strip Physics (Visual Only)** to remove the ArticulationBodies, colliders and URDF components, so the robot is only a visual hierarchy.
5. Drag the prefab under `Cube`, set position and scale as in the table above.

**Known cosmetic limits.** The wheels do not spin. If the model ever looks like it drives backwards, rotate its local Y by 180 degrees (it currently faces its direction of motion).

## Run order

Start these in separate terminals, in this order:

1. `roscore`
2. `roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000`
3. Press **Play** in Unity.
4. Your ROS nodes (see below).

## Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cube/pose` | `geometry_msgs/Pose` | Unity to ROS | 10 Hz, robot pose in the ROS frame |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS to Unity | The robot stops after 0.5 s without a command |
| `/map` | `nav_msgs/OccupancyGrid` | Unity to ROS | 1 Hz, `frame_id: map`, 0.5 m cells, 77 x 70 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | you to planner | Goal in the `map` frame |
| `/planned_path` | `nav_msgs/Path` | planner to follower | Latched |
| `/nav/cancel` | `std_msgs/Empty` | you to follower | Drops the current leg and stops the robot |
| `/nav/leg_result` | `std_msgs/String` (JSON) | planner and follower to you | `{"outcome": "succeeded", "reason": ""}` when the goal is reached. `{"outcome": "aborted", "reason": "no_path"}` when the planner finds no path, `"cancelled"` after `/nav/cancel` |

### Parameters

| Parameter | Read by | Default | Meaning |
|---|---|---|---|
| `/nav/max_lin` | follower, at every control step | 0.5 | Speed limit in m/s |
| `/nav/inflation_radius` | planner, at every plan | 0 | The planner keeps the robot centre this far from every obstacle (m). Leave it at 0 while Unity still inflates the map with Robot Radius |
| `~snap_distance` | planner | 6.0 | How far a blocked start or goal may be moved to a free cell (m) |
| `~print_map` | planner | true | Draw the plan as text in the log |
| `~waypoint_tol` | follower | 0.3 | Arrival distance for waypoints before the last one (m) |

Set one at run time with `rosparam set /nav/max_lin 0.2`.

**Frame mapping (Unity to ROS):** ROS x = Unity Z, ROS y = -Unity X, ROS yaw = -Unity rotation Y. Units are metres and radians.

## Nodes (`ros1/cube_control/scripts`)

| Script | What it does |
|---|---|
| `hello_talker.py` | Connection test: publishes a string for Unity's `HelloSubscriber` |
| `pose_listener.py` | Prints `/cube/pose` |
| `cmd_vel_publisher.py` | Open-loop 2 m square (no feedback) |
| `go_to_goal.py` | Closed-loop P-controller to a single goal |
| `map_viewer.py` | Prints `/map` as ASCII (`#` blocked, `.` free, `R` robot) |
| `astar_planner.py` | On a goal: inflates `/map` by `/nav/inflation_radius`, snaps a blocked start or goal to the nearest free cell within `~snap_distance`, runs 8-connected A* (octile heuristic), smooths the path with an exact line-of-sight, publishes `/planned_path`. Reports `aborted` / `no_path` on `/nav/leg_result` when there is no path |
| `path_follower.py` | Follows `/planned_path` waypoint by waypoint at 20 Hz within `/nav/max_lin`; stops when `/cube/pose` is older than 0.5 s; reports `succeeded` on arrival and `aborted` / `cancelled` after `/nav/cancel`; stops on shutdown; ignores a latched path that is older than its own start |

The logic of these two nodes is in plain Python modules next to them, so it can be tested without ROS: `grid_planner.py` (map, inflation, A*, line of sight), `path_tracker.py` (path state and controller) and `nav_common.py` (result JSON, parameter checks). `catkin_make` runs the nodes through a wrapper, which is why both nodes add their own folder to `sys.path`; a new module next to them is found without extra setup.

### Tests

No ROS and no Unity needed. They run on Windows and in WSL:

```bash
cd ros1/cube_control
python3 -m unittest discover -s test -v
```

## Demo: plan and drive to a goal

```bash
rosrun cube_control astar_planner.py
rosrun cube_control path_follower.py          # start BEFORE sending the goal
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Verified results (planner settings: Cell Size 0.5, Robot Radius 1, Overhead Clearance 1.5, Grid Padding 3, Snap Distance 6):
- Goal (-8, 0): 3 corners, 21.0 m, arrived at (-7.97, -0.09).
- Goal (18, 12): 4 waypoints, 31.2 m, arrived at (17.99, 11.91).

These figures were recorded with the plain cube. With the TurtleBot model attached (same settings), both goals are reached, the robot stays clear of the shelves, and it faces its direction of motion.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `python3\r: No such file or directory` | Windows line endings in a script. Convert to LF (`sed -i 's/\r$//' file.py`) |
| `Permission denied` on `rosrun` | `chmod +x` the script |
| Endpoint prints `No more data available / Disconnected` when you stop Play | Harmless |
| No topics appear in ROS | Check the start order, the ROS Settings (ROS1, 127.0.0.1:10000), and that the endpoint is running |
| Robot does not move | Make sure `CubeCarNavigator` is unticked and `CubeCmdVelSubscriber` is ticked |
| Unity Console fills with `DC[I] ... USim Time` lines | Harmless info logs from the warehouse demo's data collection |
| `rospack` prints `non-existent package 'python-...'` errors | Harmless noise from the ROS installation, not from this package |
| Git prints `LF will be replaced by CRLF` | Harmless line-ending warning on Windows |
| `Warehouse.unity` merge conflict | The scene file merges badly. Agree in the team who edits it, and edit it on one branch at a time |

## Hand-over notes

The interface to keep is `/cmd_vel` in and `/cube/pose` out.

- **Different robot size:** set **Robot Radius** on `CubeCarNavigator` to the robot's radius plus a safety margin (the grid inflation uses it), and re-tune the follower's limits (`K_LIN`, `K_ANG`, `MAX_LIN`, `MAX_ANG`) and thresholds (`TURN_FIRST`, `waypoint_tol`). For a true-size Waffle Pi, the intended values are Robot Radius about 0.3 and `MAX_LIN` about 0.22 (hardware limit 0.26 m/s linear, 1.82 rad/s angular).
- **Do not rename `Cube`** without checking the scripts and teammates' scenes: other components find it by name.

## Next steps (not done yet)

1. **Wheel-level control.** Keep the URDF's ArticulationBody wheel joints, convert `/cmd_vel` (v, w) to wheel speeds (left = (v - w*b/2)/r, right = (v + w*b/2)/r, with r = 0.033 m and b = 0.287 m), and publish the pose from the robot's own base link. This needs the true-size robot (physics bodies do not tolerate a 3.2x scale) and the smaller Robot Radius and speed limits above.
2. **Sensors.** Simulated LiDAR (`/scan`) and odometry (`/odom`) on the physical robot.
3. **Cosmetic wheel animation** (only if physics is dropped): rotate the wheel objects from the measured motion of the Cube.
