# ADR-009: Unity Owns the Clock

**Status:** Accepted

## Context
ROS nodes stamp and compare times constantly (TF lookups, AMCL, costmaps). Unity runs on Windows and ROS runs in WSL. Their wall clocks can drift apart, especially after the laptop sleeps. Unity can also run slower than real time under load.

## Decision
- Unity's `ClockPublisher` publishes `/clock` from simulation time at 100 Hz.
- All ROS nodes run with `use_sim_time: true` (set once in `bringup.launch`).
- Unity stamps every sensor message with the same simulation time.

## Consequences
- Windows↔WSL clock drift no longer matters.
- If Unity pauses, ROS time pauses too, so there are no false timeouts.
- If `/clock` stops for more than 2 s, `mission_orchestrator` treats the bridge as lost (`fail_reason = bridge_lost`).
- `duration_s` in the results is simulation time, not wall time.
