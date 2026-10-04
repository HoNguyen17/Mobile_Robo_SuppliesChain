# ADR-001: TurtleBot3 Waffle Pi, No Arm

**Status:** Accepted · amended 2026-10-04 ([ADR-013](ADR-013-kinematic-robot-body.md)). A scripted arm may replace the dwell later (manipulator feature, ADR-015)

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
- The Waffle Pi URDF and meshes are already in the repo (`Assets/URDF/`). They are used as a visual model on a kinematic body ([ADR-013](ADR-013-kinematic-robot-body.md)). Navigation is our own Python nodes ([ADR-012](ADR-012-custom-python-navigation.md)), not the TurtleBot3 configs.
- The Waffle Pi's 0.26 m/s top speed sets the Standard motion profile.

## Alternatives rejected
| Option | Why not |
|--------|---------|
| TurtleBot3 Burger | Smaller, but the Waffle Pi's model and speed limits are the ones the plan uses |
| Robot with an arm | Ungraded work that costs weeks |
