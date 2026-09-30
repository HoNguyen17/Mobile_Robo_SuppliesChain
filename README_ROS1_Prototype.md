# ROS 1 + Unity prototype: cube robot with A* navigation

A placeholder cube robot drives around the Unity warehouse (`WarehouseProjectURP`), controlled by
ROS 1 (Noetic) nodes over the Unity ROS-TCP bridge:

- Unity publishes the robot pose and an occupancy grid of the warehouse.
- A Python node plans a path with A*.
- A follower node drives the cube along that path.

**Status:** works end to end with the cube. Swapping the cube for the TurtleBot is not part of this
prototype (see [Hand-over notes](#8-hand-over-notes-for-the-turtlebot)).

---

## 1. Requirements

| Item | Version / note |
|---|---|
| Windows 10/11 with WSL2 | The ROS side runs inside WSL2 |
| WSL2 distro | **Ubuntu-20.04** (check with `lsb_release -a`). Do not use a 22.04 distro: ROS Noetic needs 20.04 |
| ROS | **Noetic** (ROS 1), `ros-noetic-desktop-full` |
| Unity | **2021.1.11f1** (URP) |
| Unity package | ROS-TCP-Connector, already in `Packages/manifest.json`. Do not upgrade it |
| ROS package on the WSL side | Unity-Technologies `ROS-TCP-Endpoint` (branch `main` is the ROS 1 version) |

---

## 2. One-time setup (WSL / ROS side)

Run these in an **Ubuntu-20.04** terminal.

```bash
# Python 3 as "python" (some scripts expect it)
sudo apt install -y python-is-python3

# Create the catkin workspace if you do not have one yet
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

# Unity's ROS <-> TCP bridge (ROS 1)
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git

# Our package: copy it from your clone of this repo (adjust the path to where you cloned it on Windows)
cp -r /mnt/c/<path-to-your-clone>/ros1/cube_control ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/cube_control/scripts/*.py

# Build
cd ~/catkin_ws
catkin_make

# Load ROS and the workspace in every new terminal
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Check
rospack find cube_control
```

After every `git pull` that changes `ros1/cube_control`, copy the package again and re-run `chmod +x`.

---

## 3. One-time setup (Unity side)

1. In Unity Hub, install **2021.1.11f1**, then *Add project* and choose `unity/WarehouseProjectURP`.
   The first open takes a while because Unity rebuilds its `Library` folder.
2. Menu **Robotics → ROS Settings**: protocol **ROS1**, ROS IP address **127.0.0.1**, port **10000**.
3. Open the scene **Warehouse**.
4. If the Console says `RosMessageTypes.Nav` does not exist: **Robotics → Generate ROS Messages…** and
   generate `nav_msgs`.

### Scene setup (already saved in the scene, listed so you know what to check)

| GameObject | Script | Ticked? | Purpose |
|---|---|---|---|
| `Cube` | `CubePosePublisher` | Yes | Publishes `/cube/pose` at 10 Hz |
| `Cube` | `CubeCmdVelSubscriber` | Yes | Moves the cube from `/cmd_vel` |
| `Cube` | `OccupancyGridPublisher` | Yes | Publishes the map on `/map` at 1 Hz |
| `Cube` | `CubeCarNavigator` | **No** | Only used to build the grid. If it is ticked while ROS drives the cube, the two fight |
| `RosHelloListener` (empty) | `HelloSubscriber` | Yes | Optional smoke test for the ROS to Unity direction |

Shelves are recognised as obstacles by the tag **`Shelf`** (`CubeCarNavigator → Obstacle Tags`).

Original stand-alone demo (no ROS): tick `CubeCarNavigator` and untick `CubeCmdVelSubscriber`, then press
Play. Press **T** to drive to the target shelf and **H** to return home.

---

## 4. Running it

**The order matters.** Use a separate Ubuntu-20.04 terminal for each numbered item.

1. `roscore`
2. `roslaunch ros_tcp_endpoint endpoint.launch tcp_ip:=0.0.0.0 tcp_port:=10000`
3. Press **Play** in Unity. The Endpoint terminal should print a connection line.
4. `rosrun cube_control astar_planner.py`
5. `rosrun cube_control path_follower.py` (start it **before** sending a goal)
6. Send a goal (ROS coordinates, metres):

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: -8.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"
```

For the current warehouse layout, `(-8, 0)` and `(18, 12)` are good test goals. The planner prints the
path as text, and the follower logs each waypoint until `Arrived at the goal`.

To see the map as text with the robot marked `R`:

```bash
rosrun cube_control map_viewer.py
```

### Smaller tests (no planner needed)

```bash
rosrun cube_control pose_listener.py        # prints x, y, yaw from /cube/pose
rosrun cube_control cmd_vel_publisher.py    # open-loop 2 m square
rosrun cube_control go_to_goal.py           # straight-line goal, no obstacle avoidance
rosrun cube_control go_to_goal.py _goal_x:=8.0 _goal_y:=-2.0
rosrun cube_control hello_talker.py         # ROS -> Unity smoke test (topic /ros_hello)
```

---

## 5. Topics

| Topic | Type | Direction | Notes |
|---|---|---|---|
| `/cube/pose` | `geometry_msgs/Pose` | Unity → ROS | 10 Hz, robot pose in the ROS frame |
| `/cmd_vel` | `geometry_msgs/Twist` | ROS → Unity | `linear.x` in m/s, `angular.z` in rad/s. The cube stops if nothing arrives for 0.5 s |
| `/map` | `nav_msgs/OccupancyGrid` | Unity → ROS | 1 Hz, 0.5 m cells, obstacles already inflated by *Robot Radius*, frame `map` |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | you → planner | Only the position is used |
| `/planned_path` | `nav_msgs/Path` | planner → follower | Latched. The follower ignores paths older than its own start |
| `/ros_hello` | `std_msgs/String` | ROS → Unity | Smoke test |

**Frame mapping (Unity → ROS):** ROS x = Unity Z, ROS y = −Unity X, ROS yaw = −Unity rotation Y.
Heading 0 faces ROS +x, positive `angular.z` turns left. Units are metres and radians.

---

## 6. Nodes (`ros1/cube_control/scripts/`)

| Script | What it does |
|---|---|
| `hello_talker.py` | Publishes a string on `/ros_hello` |
| `pose_listener.py` | Prints the pose from `/cube/pose` |
| `cmd_vel_publisher.py` | Open-loop 2 m square via `/cmd_vel` |
| `go_to_goal.py` | Proportional controller to one goal, ignores obstacles |
| `astar_planner.py` | Reads `/map` + `/cube/pose`, plans on a goal, publishes `/planned_path` |
| `path_follower.py` | Drives along `/planned_path` waypoint by waypoint |
| `map_viewer.py` | Prints `/map` as text with the robot marked |

---

## 7. Troubleshooting

| Problem | Fix |
|---|---|
| `roscore` or `rosrun` not found | Wrong distro or `.bashrc` not loaded. Check `lsb_release -a` says 20.04, then `source ~/.bashrc` |
| Unity cannot connect | Is the Endpoint running? Check **Robotics → ROS Settings** (ROS1, 127.0.0.1, 10000) |
| Endpoint prints `Exception: No more data available` then `Disconnected` | Harmless. It appears every time you stop Play. No restart needed |
| `No /map received yet` in the planner | Unity is not in Play mode, `OccupancyGridPublisher` is unticked, or wait 1 s after pressing Play |
| Cube does not move | Check `rostopic echo /cmd_vel`. Make sure `CubeCmdVelSubscriber` is ticked and `CubeCarNavigator` is **unticked** |
| Cube jitters or turns randomly | Two controllers are active. Untick `CubeCarNavigator` and stop other nodes publishing `/cmd_vel` |
| Follower says `Ignoring an old (latched) path` | Expected on start. Send a new goal after the follower is running |
| Planner warns `snapped to` | The goal was inside a shelf or blocked area. It moved to the nearest reachable free cell within `~snap_distance` (default 6 m) |
| `bad interpreter: /usr/bin/env: 'python3\r'` | Windows line endings. Fix: `sed -i 's/\r$//' ~/catkin_ws/src/cube_control/scripts/*.py` |
| `Permission denied` when running a script | `chmod +x ~/catkin_ws/src/cube_control/scripts/*.py` |

---

## 8. Hand-over notes for the TurtleBot

To swap the cube for the TurtleBot without touching the ROS nodes, keep this interface:

- The Unity robot must **subscribe to `/cmd_vel`** (Twist) and **publish `/cube/pose`** (Pose of the robot
  centre, in the ROS frame as above). The topic names can be changed later, but then change them in the
  ROS scripts as well.
- Set **`Robot Radius`** on `CubeCarNavigator` to the TurtleBot's radius plus a safety margin (check the
  model's real footprint). The map is inflated by this value, and the planner treats the robot as a point.
- Re-tune the speed limits in `path_follower.py` (`MAX_LIN`, `MAX_ANG`, and the gains) to the TurtleBot's
  limits. The cube's values are only a starting point.
- `CubeCarNavigator` stays on the robot but unticked: `OccupancyGridPublisher` only uses it to build the grid.
- Stop conditions: keep the 0.5 s no-command watchdog in the Unity subscriber.

---

## 9. Repository layout

```
ros1/cube_control/          ROS 1 package (package.xml, CMakeLists.txt, scripts/)
unity/WarehouseProjectURP/  Unity project (Assets/, Packages/, ProjectSettings/)
README_ROS1_Prototype.md    this file
```

`ROS-TCP-Endpoint` is not stored here. Clone it as described in section 2.

---

## 10. Upstream license

The warehouse scene in `unity/WarehouseProjectURP` is derived from
[Unity-Technologies/Robotics-Warehouse](https://github.com/Unity-Technologies/Robotics-Warehouse),
licensed under the **Apache License 2.0**. See [`unity/UPSTREAM_LICENSE`](unity/UPSTREAM_LICENSE).
