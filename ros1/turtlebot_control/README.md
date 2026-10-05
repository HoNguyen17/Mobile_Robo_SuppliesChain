# turtlebot_control

The navigation nodes of the UC6 warehouse robot ([ADR-012](../../docs/ADR-012-custom-python-navigation.md)):
an A* planner and a path follower for the physical TurtleBot3 Waffle Pi in Unity. Same algorithm and
structure as the earlier `cube_control` package, now in true robot metres, on simulation time.

| Node | Job |
|------|-----|
| `astar_planner.py` | On a goal: inflates the raw `/map`, snaps a blocked start or goal to a free cell, runs A*, smooths the path, publishes `/planned_path`. Reports `aborted` / `no_path` on `/nav/leg_result` when there is none |
| `path_follower.py` | Follows `/planned_path` at 20 Hz, publishes `/cmd_vel`. Stops when `/robot/pose` is older than 0.5 s. Reports `succeeded` on arrival, `aborted` / `cancelled` after `/nav/cancel` or when Unity restarts its clock |

The logic is in plain Python modules without `rospy`, so it is unit tested anywhere:
`grid_planner.py` (map, inflation, A*, line of sight), `path_tracker.py` (path state and controller) and
`nav_common.py` (result JSON, parameter checks, waiting for `/clock`).

## Topics and parameters

| Topic | Type | Direction |
|-------|------|-----------|
| `/map` | `nav_msgs/OccupancyGrid` | Unity to planner. Raw (not inflated), 0.05 m cells, frame `map` |
| `/robot/pose` | `geometry_msgs/PoseStamped` | Unity to planner and follower. Ground truth, frame `map`, simulation time |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | you (or RViz *2D Nav Goal*) to planner. Frame `map` |
| `/planned_path` | `nav_msgs/Path` | planner to follower. Latched |
| `/cmd_vel` | `geometry_msgs/Twist` | follower to Unity |
| `/nav/cancel` | `std_msgs/Empty` | you to follower |
| `/nav/leg_result` | `std_msgs/String` (JSON) | planner and follower to you: `{"outcome": "succeeded"|"aborted", "reason": ""|"no_path"|"cancelled"}` |

| Parameter | Read by | Default | Meaning |
|-----------|---------|---------|---------|
| `/nav/max_lin` | follower, every step | 0.26 | Speed limit (m/s) |
| `/nav/inflation_radius` | planner, every plan | 0.35 | The robot centre stays this far from every obstacle (m). The footprint reaches 0.257 m from the wheel axis centre |
| `~snap_distance` | planner | 2.0 | How far a blocked start or goal may move to a free cell (m) |
| `~waypoint_tol` | follower | 0.15 | Arrival distance for corners before the last (m) |

## Build and run (WSL, Ubuntu 20.04, ROS Noetic)

Use a workspace of its own, so the older packages in `~/catkin_ws` are not touched:

```bash
mkdir -p ~/nav_ws/src
```

```bash
ln -s "/mnt/c/<path to the repo>/ros1/turtlebot_control" ~/nav_ws/src/turtlebot_control
```

```bash
ln -s "/mnt/c/<path to the repo>/ros1/warehouse_bringup" ~/nav_ws/src/warehouse_bringup
```

```bash
cd ~/nav_ws && catkin_make
```

In every terminal: `source ~/nav_ws/devel/setup.bash`. Then, with Unity not yet playing:

```bash
roslaunch warehouse_bringup bringup.launch
```

Press Play in Unity. Send a goal and watch the result:

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

```bash
rostopic echo /nav/leg_result
```

Other things to try: `rosparam set /nav/max_lin 0.15`, `rostopic pub -1 /nav/cancel std_msgs/Empty "{}"`.
`roslaunch warehouse_bringup bringup.launch nav:=false` starts only the bridge, the robot model and RViz.

## Tests

No ROS and no Unity needed. They run on Windows and in WSL:

```bash
cd ros1/turtlebot_control
python3 -m unittest discover -s test -v
```

`test_closed_loop.py` plans on a map and drives a simulated Waffle Pi (speed limits, acceleration ramp, wheel slip,
30 Hz pose with latency) with the real follower logic, and checks that the footprint circle (0.257 m) never touches a wall.
