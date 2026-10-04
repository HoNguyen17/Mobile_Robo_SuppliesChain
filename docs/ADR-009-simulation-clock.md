# ADR-009: Unity Owns the Clock

**Status:** Accepted · amended 2026-10-04 ([ADR-013](ADR-013-kinematic-robot-body.md)): the pose is stamped too, and there is no TF

## Context
ROS nodes stamp and compare times constantly (pose and scan stamps, leg timeouts, metrics). Unity runs on Windows and ROS runs in WSL. Their wall clocks can drift apart, especially after the laptop sleeps. Unity can also run slower than real time under load, and the kinematic robot moves by Unity's own frame time.

## Decision
- Unity's `ClockPublisher` publishes `/clock` from simulation time at 100 Hz. It is ported from `Nguyen-planning` in P1.
- All ROS nodes run with `use_sim_time: true` (set once in `bringup.launch`).
- Unity stamps `/cube/pose` (`PoseStamped`) and every sensor message with the same simulation time.

## Consequences
- Windows↔WSL clock drift no longer matters.
- If Unity pauses, ROS time pauses too, so there are no false timeouts.
- If `/clock` stops for more than 2 s, the `mission` node treats the bridge as lost (`fail_reason = bridge_lost`).
- `duration_s` in the results is simulation time, not wall time.
- A node must wait for the first `/clock` message before it reads `rospy.Time.now()`. Until then the time is 0.
