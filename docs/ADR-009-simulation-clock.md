# ADR-009: Unity Owns the Clock

**Status:** Accepted · amended 2026-10-04 ([ADR-015](ADR-015-physical-waffle-pi-body.md)): the pose and the path are stamped with simulation time, the follower waits for the first `/clock` and survives a Unity restart

## Context
ROS nodes stamp and compare times constantly (pose and scan stamps, leg timeouts, metrics). Unity runs on Windows and ROS runs in WSL. Their wall clocks can drift apart, especially after the laptop sleeps. Unity can also run slower than real time under load, and the robot's physics runs on Unity's own time.

## Decision
- Unity's `ClockPublisher` (`Assets/Scripts/Ros/`) publishes `/clock` (`rosgraph_msgs/Clock`) from simulation time, once per rendered frame, before any other script runs in that frame. It never sends the same or an older time twice.
- All ROS nodes run with `use_sim_time: true` (`bringup.launch`; `navigation.launch` sets it too).
- Unity stamps `/robot/pose` (`PoseStamped`), the TF `map -> base_footprint`, `/odom`, `/scan` and `/map` with the same simulation time. The planner stamps `/planned_path` with ROS time, which is that simulation time.
- `path_follower` waits for the first `/clock` message before it starts (`wait_for_clock`).
- **Unity restart.** When the person presses Play again, the clock goes back to 0. `path_follower` sees the time move back by more than 1 s, drops its old pose, and ends a running leg as `aborted` with reason `cancelled` on `/nav/leg_result`. The nodes keep running and take the next goal; they do not need a restart.

## Consequences
- Windows↔WSL clock drift no longer matters.
- If Unity pauses, ROS time pauses too, so there are no false timeouts.
- If `/clock` stops for more than 2 s, the `mission` node treats the bridge as lost (`fail_reason = bridge_lost`).
- `duration_s` in the results is simulation time, not wall time.
- A node must wait for the first `/clock` message before it reads `rospy.Time.now()`. Until then the time is 0, and a `rospy.Rate` would wait forever. `path_follower` does this; the planner only reads the time when a goal arrives.
- The stamps of `/robot/pose` and `/planned_path` are in the same time base, so the follower's age check (the robot stops when the pose is older than 0.5 s) and the planner's (it refuses to plan from a pose older than 1 s) work in simulation seconds.
- A restarted follower ignores a latched `/planned_path` that is older than the node itself (checked for the first 3 s only, so new paths after a Unity restart are never taken for old ones).
