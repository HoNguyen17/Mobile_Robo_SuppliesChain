# Archived: the ROS side of the Cube prototype

Replaced by `ros1/turtlebot_control` ([ADR-015](../../docs/ADR-015-physical-waffle-pi-body.md)), which keeps the algorithm and structure of this package in true robot metres on simulation time. Moved here, not deleted, after the acceptance run passed on 2026-10-05.

| Item | What it was |
|------|-------------|
| `cube_control/` | ROS 1 package of the `Nhan-turtlebot` prototype: A* planner, path follower, leg result and cancel, 85 unit tests. Its `scripts/` also hold the prototype's smoke-test nodes (`hello_talker`, `pose_listener`, `map_viewer`, `go_to_goal`, `cmd_vel_publisher`) |
| `README_ROS1_Prototype.md` | Step-by-step guide of the prototype (Cube in Unity, scale 3.2). Its paths refer to the old locations |

The package is not part of any workspace now. To run it again, link it into a catkin workspace (`ln -s <repo>/_archive/ros1-cube-prototype/cube_control ~/some_ws/src/cube_control`). The Unity side is in [`_archive/unity-cube-prototype/`](../unity-cube-prototype/README.md).
