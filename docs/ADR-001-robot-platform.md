# ADR-001: TurtleBot3 Waffle Pi, No Arm

**Status:** Accepted · amended 2026-10-04 ([ADR-015](ADR-015-physical-waffle-pi-body.md)): the robot is the physical Waffle Pi in Unity, not a kinematic stand-in. A scripted arm may replace the dwell later (manipulator feature, its own ADR)

## Context
The robot must "retrieve" items, but the grade is for obstacle avoidance, not manipulation.

## Decision
Use a **TurtleBot3 Waffle Pi** without an arm. Retrieval works like this:

1. Drive to the item's known shelf pose.
2. Dwell `dwell_s` seconds.
3. Unity re-parents the item to the robot (`/sim/attach_item`, answered on `/sim/ack`).

Release at drop-off is the reverse (`/sim/release_item`).

## Consequences
- No grasping, arm kinematics or MoveIt.
- The Waffle Pi URDF and meshes are in the repo (`Assets/URDF/`). Unity imports them as a **physical robot**: articulated wheels driven by `DiffDriveController`, robot scale 4 ([ADR-015](ADR-015-physical-waffle-pi-body.md), [ADR-010](ADR-010-robot-scale.md)). Navigation is our own Python nodes ([ADR-012](ADR-012-custom-python-navigation.md)), not the TurtleBot3 configs.
- The Waffle Pi's limits, 0.26 m/s linear and 1.82 rad/s angular, set the Standard motion profile.
- Sensors in the simulation: the LDS-01 LIDAR as raycasts (`/scan`) and odometry read from the simulated body (`/odom`); the pose and the map are ground truth ([ADR-002](ADR-002-perception-scope.md)). The URDF also has the camera and the IMU as links, but neither is simulated.
- A physical body can slide and tip. A mast or arm added later by the manipulator feature can upset its balance, so that feature must re-check the physics ([ADR-011](ADR-011-physics-standard.md)).

## Alternatives rejected
| Option | Why not |
|--------|---------|
| TurtleBot3 Burger | Smaller, but the Waffle Pi's model and speed limits are the ones the plan uses |
| Robot with an arm | Ungraded work that costs weeks |
