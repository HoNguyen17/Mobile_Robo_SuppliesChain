# ROS1 + Unity prototype: cube robot with A* navigation

A cube robot drives through the Unity warehouse scene (`WarehouseProjectURP`). Unity and ROS1 talk over ROS-TCP-Endpoint. ROS plans a path with A* on an occupancy grid published by Unity, and a follower node sends velocity commands back.

The course requires **ROS1** (not ROS 2).

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
4. On the Cube object, tick these scripts: `CubePosePublisher`, `CubeCmdVelSubscriber`, `OccupancyGridPublisher`.
5. Leave `CubeCarNavigator` **unticked** while ROS drives the cube. It is the original Unity-side planner and would fight the ROS commands.

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
| `/cmd_vel` | `geometry_msgs/Twist` | ROS to Unity | The cube stops after 0.5 s without a command |
| `/map` | `nav_msgs/OccupancyGrid` | Unity to ROS | 1 Hz, `frame_id: map`, 0.5 m cells, 77 x 70 |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | you to planner | Goal in the `map` frame |
| `/planned_path` | `nav_msgs/Path` | planner to follower | Latched |

**Frame mapping (Unity to ROS):** ROS x = Unity Z, ROS y = -Unity X, ROS yaw = -Unity rotation Y. Units are metres and radians.

## Nodes (`ros1/cube_control/scripts`)

| Script | What it does |
|---|---|
| `hello_talker.py` | Connection test: publishes a string for Unity's `HelloSubscriber` |
| `pose_listener.py` | Prints `/cube/pose` |
| `cmd_vel_publisher.py` | Open-loop 2 m square (no feedback) |
| `go_to_goal.py` | Closed-loop P-controller to a single goal |
| `map_viewer.py` | Prints `/map` as ASCII (`#` blocked, `.` free, `R` robot) |
| `astar_planner.py` | 8-connected A* (octile heuristic) on `/map`; snaps an unreachable goal to the nearest free cell (up to about 6 m); smooths the path with line-of-sight |
| `path_follower.py` | Follows `/planned_path` waypoint by waypoint at 20 Hz; stops on shutdown; ignores latched paths older than its own start |

## Demo: plan and drive to a goal

```bash
rosrun cube_control astar_planner.py
rosrun cube_control path_follower.py          # start BEFORE sending the goal
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: -8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

Verified results:
- Goal (-8, 0): 3 corners, 21.0 m, arrived at (-7.97, -0.09).
- Goal (18, 12): 4 waypoints, 31.2 m, arrived at (17.99, 11.91).

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `python3\r: No such file or directory` | Windows line endings in a script. Convert to LF (`sed -i 's/\r$//' file.py`) |
| `Permission denied` on `rosrun` | `chmod +x` the script |
| Endpoint prints `No more data available / Disconnected` when you stop Play | Harmless |
| No topics appear in ROS | Check the start order, the ROS Settings (ROS1, 127.0.0.1:10000), and that the endpoint is running |
| Robot does not move | Make sure `CubeCarNavigator` is unticked and `CubeCmdVelSubscriber` is ticked |
| `rospack` prints `non-existent package 'python-...'` errors | Harmless noise from the ROS installation, not from this package |

## Hand-over notes for the TurtleBot step

The interface to keep is `/cmd_vel` in and `/cube/pose` out. For a different robot:
- Set **Robot Radius** on `CubeCarNavigator` to the robot's radius plus a safety margin. The grid inflation uses it.
- Re-tune the follower's speed limits (`K_LIN`, `K_ANG`, `MAX_LIN`, `MAX_ANG`) and thresholds (`TURN_FIRST`, `waypoint_tol`).
